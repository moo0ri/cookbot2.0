#importar bibliotecas necessárias
import numpy as np

#importar parâmetros do robô
from parametros_cookbot import cookbot # type: ignore
from traj_cookbot import traj_des # type: ignore

#posição de teste
pos_test = (0.17, np.pi/2, np.deg2rad(10))

#plot com sliders
cookbot.teach(pos_test)

#plot da trajetória
cookbot.plot(traj_des.q)


