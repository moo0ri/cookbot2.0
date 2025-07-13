#PARÂMETROSROBÔ

# Import das bibliotecas necessárias

from roboticstoolbox import ERobot, Link, ET
import numpy as np

# Definindo os parâmetros do robô Cookbot

a2 = 0.6
m0 = 0.816
m1 = 1.094
m2 = 1.958
m3 = 0.941
g = 9.81

I0 = np.diag([0.004653, 0.004743, 0.003209])
I1 = np.diag([0.01206, 0.01217, 0.001835])
I2 = np.diag([0.005, 0.099, 0.101])
I3 = np.diag([0.01702, 0.01727, 0.005741])

# Definindo o manipulador robótico
link0 = Link(ET.tz(qlim=[0, 0.170]), r = [0.003222, 0.000155, 0.046056], m = m0, I = I0, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link1 = Link(ET.tz(0.300) * ET.Rz(qlim=[-np.pi, np.pi]), r = [0.002836, 0.000116, 0.110482], m = m1, I = I1, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link2 = Link(ET.tx(a2) * ET.Rx(qlim=[-np.pi, np.pi]), r = [0.266, 0, 0.034], m = m2, I = I2, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link3 = Link(ET.tx(0.05) * ET.Ry(-np.deg2rad(30)) * ET.tx(0.05773) * ET.tz(-0.102) * ET.tz(0.148), r = [0, 0.04039, 0.127848], m = m2, I = I2, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)

cookbot = ERobot([link0, link1, link2, link3], name="Cookbot")

