from parametros_cookbot import cookbot
from traj_cookbot import traj_des
import numpy as np

q_real = np.load(r'C:\Users\User\Desktop\Cookbot2.0\trajetoria_q_sim.npy')

cookbot.plot(q_real.T) 
