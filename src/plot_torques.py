#PLOT TORQUES
from matplotlib import pyplot as plt
from traj_cookbot import t_des, Torque_traj

t = t_des

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axs[0].plot(t, Torque_traj[:, 0], label='Torque Junta 1')
axs[0].set_ylabel('Torque [Nm]')
axs[0].set_title('Torque da Junta 1 ao Longo do Tempo')
axs[0].grid(True)

axs[1].plot(t, Torque_traj[:, 1], label='Torque Junta 2', color='orange')
axs[1].set_ylabel('Torque [Nm]')
axs[1].set_title('Torque da Junta 2 ao Longo do Tempo')
axs[1].grid(True)

axs[2].plot(t, Torque_traj[:, 2], label='Torque Junta 3', color='green')
axs[2].set_xlabel('Amostra')
axs[2].set_ylabel('Torque [Nm]')
axs[2].set_title('Torque da Junta 3 ao Longo do Tempo')
axs[2].grid(True)

plt.tight_layout()
plt.show()
