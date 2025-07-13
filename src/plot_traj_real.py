from parametros_cookbot import cookbot
from traj_cookbot import traj_des
import numpy as np
import os

# Caminho absoluto para a pasta 'data'
data_path = os.path.join(os.path.dirname(__file__), '..', 'data', 'trajetoria_q_sim.npy')
q_sim = np.load(data_path)


print(q_sim.shape)  # Só para conferir o shape do array

cookbot.plot(q_sim.T) 
