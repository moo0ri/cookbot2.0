#TRAJETÓRIA COOKBOT

#importar parâmetros do robô
from parametros_cookbot import cookbot, g # type: ignore
import numpy as np
from roboticstoolbox import mstraj

frit = np.deg2rad(8)
frit_desp = np.deg2rad(280)

#TRAJETÓRIAS
#batatas serão despejadas na bandeja
POS0 = cookbot.fkine([0, 0, frit])
#posição 1: batatas acima da fritadeira
POS1 = cookbot.fkine([0.17, np.pi/2, frit])
#posição 2: batatas  dentro da fritadeira
POS2 = cookbot.fkine([0, np.pi/2, frit])
#posição 3: retirar as batatas da fritadeira
POS3 = POS1
#posição 4: levar as batatas até a bandeja de despejo
POS4 = cookbot.fkine([0.17, np.pi, frit])
#posição 5: despejar as batatas na bandeja
POS5 = cookbot.fkine([0.17, np.pi, frit_desp])

#Crinado um vetor com as posições
viapoints = np.array([
    [0, 0, np.deg2rad(10)],                       # POS0
    [0.17, np.pi/2, 0],                           # POS1
    [0, np.pi/2, 0],                              # POS2
    [0.17, np.pi/2, 0],                           # POS3
    [0.05, np.pi, np.deg2rad(45)],                # POS4
    [0.05, np.pi, np.deg2rad(270)],               # POS5
])

#vetor com os tempos de cada segmento
tsegment = np.array([5,         # Tempo para ir de POS0 a POS1                    
                     8,         # Tempo para ir de POS1 a POS2  
                     8,         # Tempo para ir de POS2 a POS3
                     6,         # Tempo para ir de POS3 a POS4
                     5])        # Tempo para ir de POS4 a POS5

traj_des = mstraj(
    viapoints=viapoints, 
    dt=0.04, 
    tacc=2, 
    tsegment = tsegment
)

q_des = traj_des.q
t_des = traj_des.t

# Derivada numérica para obter velocidade
qd_des = np.gradient(q_des, t_des, axis=0)

# Derivada numérica para obter aceleração
qdd_des = np.gradient(qd_des, t_des, axis=0)

#TORQUE NA TRAJETÓRIA

# Cálculo do torque ideal usando a função rne (Robotic Newton-Euler)
Torque_traj = cookbot.rne(q_des, qd_des, qdd_des, gravity=[0, 0, g], symbolic=False)

