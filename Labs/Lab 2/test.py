import numpy as np
import pandas as pd
import os
from matplotlib import pyplot as plt
from sympy import Symbol
from PySpice.Spice.Netlist import Circuit, SubCircuitFactory
from PySpice.Spice.Library import SpiceLibrary
from PySpice.Plot.BodeDiagram import bode_diagram
from scipy import signal, optimize
import shutil
import tempfile
import PySpice.Logging.Logging as Logging
logger = Logging.setup_logging(logging_level='DEBUG')

# Janky monkeypatch hack
from PySpice.Spice.NgSpice.Shared import NgSpiceShared
def exec_command(self, command, join_lines=True):

        """ Execute a command and return the output. """

        if len(command) > self.__MAX_COMMAND_LENGTH__:
            raise ValueError('Command must not exceed {} characters'.format(self.__MAX_COMMAND_LENGTH__))

        self._logger.debug('Execute command: {}'.format(command))
        self.clear_output()
        rc = self._ngspice_shared.ngSpice_Command(command.encode('ascii'))
        if rc:
            raise NameError("ngSpice_Command '{}' returned {}".format(command, rc))
        if join_lines:
            return self.stdout
        else:
            return self._stdout

NgSpiceShared.exec_command = exec_command

class PA13Amplifier(SubCircuitFactory):
    __name__ = 'pa13_amplifier'
    __nodes__ = ('in+', 'out')
    def __init__(self, Z_1, Z_2, R_lim=0.2):
        super().__init__()

        # Supply rails
        self.V('s+', 'vs+', self.gnd, 25)
        self.V('s-', self.gnd, 'vs-', 25)

        # PA13
        self.X('1', 'PA13',
                'in+', 'in-', 'out',  # Inputs/outputs
                'vs+', 'vs-',  # Supply rails
                'cl+', 'cl-',  #  Current limit
                self.gnd)  # foldover

        # Current limiting resistors
        self.R('lim+', 'cl+', 'out', R_lim)
        self.R('lim-', 'cl-', 'out', R_lim)

        # Gain setting resistors
        self.R('z1', self.gnd, 'in-', Z_1)
        self.R('z2', 'in-', 'out', Z_2)

class OP27Amplifier(SubCircuitFactory):
    __name__ = 'op27_amplifier'
    __nodes__ = ('in-', 'out')
    def __init__(self, R_4, C_4):
        super().__init__()

        # Supply rails
        self.V('s+', 'vs+', self.gnd, 25)
        self.V('s-', self.gnd, 'vs-', 25)

        # PA13
        self.X('1', 'OP27',
                self.gnd, 'in-',  # Inputs
                'vs+', 'vs-',  # Supply rails
                'out') # Output

        # Gain setting resistors
        self.R('r4', 'in-', 'f41', R_4)
        self.C('c4', 'f41', 'out', C_4)

# Values from prelab/schematic
Z_1 = 2.2*10**3  # ohm
Z_2 = 22*10**3  # ohm
Z_3 = 10*10**3  # ohm
R_4 = 56*10**3  # ohm
C_4 = 2.7*10**-9 # F
Z_5 = 2*10**3  # ohm
R_m = 3  # ohm
L_m = 1.07*10**-3  # H
R_s = 0.2  # ohm

def step_resp(vpk_pk):
    vmax = vpk_pk/2
    shutil.copyfile('PA13.LIB', '/tmp/PA13.LIB')
    shutil.copyfile('op27.cir', '/tmp/op27.cir')
    circuit = Circuit('Step Response')
    circuit.include('/tmp/PA13.LIB')
    circuit.include('/tmp/op27.cir')
    circuit.subcircuit(PA13Amplifier(Z_1, Z_2))
    circuit.subcircuit(OP27Amplifier(R_4, C_4))

    # V_Ir input
    circuit.PulseVoltageSource(
        'input', 'vir', circuit.gnd,
        -vmax, vmax,
        pulse_width=0.05,  # 0.05s
        period=0.1,  # 0.1s, 10Hz
        rise_time=1e-9, fall_time=1e-9)  # From lab provided schematic
    circuit.R('r3', 'vir', 'vir2', Z_3)
    # Current controller stage
    circuit.X('curr', 'op27_amplifier', 'vir2', 'vr')
    # Voltage stage
    circuit.X('volt', 'pa13_amplifier', 'vr', 'vo')
    # Motor stage
    circuit.R('rm', 'vo', 'vm1', R_m)
    circuit.L('lm', 'vm1', 'vio', L_m)

    circuit.R('rs', 'vio', circuit.gnd, R_s)
    circuit.R('r5', 'vio', 'vir2', Z_5)

    simulator = circuit.simulator()
    # Force ngspice to have shorter time steps near sharp transitions
    simulator.options(trtol=0.00001)  
    import pdb; pdb.set_trace()
    analysis = simulator.transient(
        step_time=0.1,  # Lab schematic suggests 0.1s, this seems too slow
        end_time=0.045)  # Lab schematic stops at 1s, we only need the first step
    return analysis

step_resp(0.1)

omega_h = 6.64*10**4  # rad/s
f_h = omega_h/(2*np.pi)  # Hz
def freq_resp():
    shutil.copyfile('PA13.LIB', '/tmp/PA13.LIB')
    shutil.copyfile('op27.cir', '/tmp/op27.cir')
    circuit = Circuit('Freq Response')
    circuit.include('/tmp/PA13.LIB')
    circuit.include('/tmp/op27.cir')
    circuit.subcircuit(PA13Amplifier(Z_1, Z_2))
    circuit.subcircuit(OP27Amplifier(R_4, C_4))

    # V_Ir input
    circuit.SinusoidalVoltageSource(
        'input', 'vr', circuit.gnd, amplitude=0.05)
    circuit.R('r3', 'vir', 'vir2', Z_3)
    # Current controller stage
    circuit.X('curr', 'op27_amplifier', 'vir2', 'vr')
    # Voltage stage
    circuit.X('volt', 'pa13_amplifier', 'vr', 'vo')
    # Motor stage
    circuit.R('rm', 'vo', 'vm1', R_m)
    circuit.L('lm', 'vm1', 'vio', L_m)

    circuit.R('rs', 'vio', circuit.gnd, R_s)
    circuit.R('r5', 'vio', 'vir2', Z_5)

    simulator = circuit.simulator()
    # Force ngspice to have shorter time steps near sharp transitions
    simulator.options(trtol=0.0001)  
    import pdb; pdb.set_trace()
    analysis = simulator.ac(
        start_frequency=50e0,
        stop_frequency=5*f_h,
        number_of_points=1000,  # Lab manual suggests 20, might as well do more
        variation='dec')
    return analysis
an2 = freq_resp()
print(an2)
