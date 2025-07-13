#PLOT TRAJETÓRIAS
import matplotlib.pyplot as plt
import numpy as np

from parametros_cookbot import cookbot # type: ignore
from traj_cookbot import q_des, qd_des, qdd_des ,t_des, traj_des #type: ignore

q = q_des
qd = qd_des
qdd = qdd_des
t = t_des

# Plota posições, velocidades e acelerações em subplots na mesma figura
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Posições (q)
for i in range(q.shape[1]):
    axs[0].plot(t, q[:, i], label=f'q{i}')
axs[0].set_title('Posição das Juntas (q)')
axs[0].set_ylabel('Posição [rad]')
axs[0].legend()
axs[0].grid(True)

# Velocidades (qd)
for i in range(q.shape[1]):
    axs[1].plot(t, qd[:, i], label=f'q{i}')
axs[1].set_title('Velocidade das Juntas (qd)')
axs[1].set_ylabel('Velocidade [rad/s]')
axs[1].legend()
axs[1].grid(True)

# Acelerações (qdd)
for i in range(q.shape[1]):
    axs[2].plot(t, qdd[:, i], label=f'q{i}')
axs[2].set_title('Aceleração das Juntas (qdd)')
axs[2].set_xlabel('Tempo [s]')
axs[2].set_ylabel('Aceleração [rad/s²]')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()

