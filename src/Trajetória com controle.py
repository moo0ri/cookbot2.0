import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from parametros_cookbot import cookbot
from traj_cookbot import traj_des, q_des, qd_des, qdd_des, t_des

# Ganhos PID encontrados
Kp = np.array([20, 40, 30]) 
Kd = np.array([5, 10, 7.5])     
Ki = np.array([0, 0, 0])   


# Mudança do nome das variáveis da trajetória desejada
t_span = t_des  
qd_values = q_des
dqd_values = qd_des
ddqd_values = qdd_des

#Função que define a dinâmica do robô e o controlador PID, projetada para ser usada com um resolvedor de EDOs como solve_ivp.
def dynamics(t, y, t_des, q_des, qd_des, qdd_des, Kp, Kd, Ki):
    
    # Extração dos estados atuais
    q = y[0:3]
    dq = y[3:6]
    integral_e = y[6:9] # O erro integral é agora parte do estado

    # Definição das matrizes de inércia vindas do parametros_cookbot
    C = cookbot.coriolis(q, dq)  # Matriz de Coriolis
    G = np.zeros(0, 0, 9.81)     # Gravidade 
    M = cookbot.inertia(q)       # Matriz de inércia


    # Adicionando verificação de limites para garantir que t esteja dentro do intervalo de t_des
    t_clamped = np.clip(t, t_des[0], t_des[-1])

    qd_t = np.array([np.interp(t_clamped, t_des, q_des[:, i]) for i in range(q_des.shape[1])])
    dqd_t = np.array([np.interp(t_clamped, t_des, qd_des[:, i]) for i in range(qd_des.shape[1])])

    # Calcula os erros
    e = qd_t - q
    de = dqd_t - dq

    # Calcula o torque de controle PID
    tau = Kp * e + Kd * de + Ki * integral_e
    
    # --- Debugging Prints --- Utilizado para verificar se haviam valores que explodiam para inf ou nan
    #print(f"Time: {t:.4f}")
    #print(f"q: {q}")
    #print(f"dq: {dq}")
    #print(f"e: {e}")
    #print(f"de: {de}")
    #print(f"integral_e: {integral_e}")
    #print(f"tau: {tau}")
    #print(f"M: {M}")
    #print(f"C: {C}") 
    #print(f"tau - C @ dq - G: {tau - C @ dq - G}")
    # --------------------------
    
    # Verifica se M é uma matriz 3x3 e não algo inesperado
    if M.shape != (3, 3):
         print(f"Unexpected shape for inertia matrix M: {M.shape} at time {t}, q={q}. Stopping integration.")
         return np.full_like(y, np.nan)

    # Verifica o número de condição da matriz de inércia para detectar singularidades
    # Um número de condição alto indica que a matriz está próxima de ser singular
    try:
        cond_M = np.linalg.cond(M)
        if cond_M > 1e10: # Limiar para considerar a matriz mal condicionada/singular
            print(f"Singularity or near-singularity detected at time {t:.4f}, q={q}. Condition number of M: {cond_M:.2e}. Stopping integration.")
            return np.full_like(y, np.nan)
    except np.linalg.LinAlgError:
        # numpy.linalg.cond pode levantar LinAlgError para matrizes que já são singulares
        print(f"Singular matrix encountered while checking condition number at time {t:.4f}, q={q}. Stopping integration.")
        return np.full_like(y, np.nan)

    try:
        ddq = np.linalg.solve(M, tau - C @ dq - G)
    except np.linalg.LinAlgError:
        print(f"Singular matrix encountered at time {t:.4f}, q={q}. Stopping integration.")
        return np.full_like(y, np.nan)


    # A derivada do erro integral é o próprio erro de posição
    de_integral = e

    # --- Anti-windup para o erro integral ---
    # Limita o erro integral para evitar que cresça indefinidamente
    integral_e_limit = 100.0 # Defina um limite razoável
    integral_e_new = integral_e + de_integral * (t - y[-1]) # Calcular o próximo valor antes de aplicar o limite
    integral_e_new = np.clip(integral_e_new, -integral_e_limit, integral_e_limit)
    de_integral_clipped = integral_e_new - integral_e # A derivada real após clipping


    # Derivadas do vetor de estado [q, dq, integral_e] são [dq, ddq, e]
    # Atualizar para usar a derivada do erro integral com anti-windup
    dy = np.concatenate([dq, ddq, de_integral_clipped])
    return dy

def run_pid_control():

    q0 = q_des[0, :]  # Posição inicial igual ao primeiro ponto da trajetória desejada
    dq0 = np.zeros(3) # Velocidade inicial zero
    integral_e0 = np.zeros(3) # Erro integral inicial zero
    y0 = np.concatenate([q0, dq0, integral_e0])


    t_span_sim = [t_des[0], 31.0] # Intervalo de tempo para solve_ivp
    t_eval_sim = t_des[t_des <= 31.0] # Pontos de tempo onde queremos a solução

    # Simulação usando solve_ivp

    # Passando os dados da trajetória e os ganhos PID como argumentos adicionais
    sol = solve_ivp(dynamics,
                    t_span_sim,
                    y0,
                    t_eval=t_eval_sim,
                    args=(t_des, q_des, qd_des, qdd_des, Kp, Kd, Ki), # Passando os argumentos adicionais
                    method='Radau') # Tentando um resolvedor para sistemas rígidos
    # Verifica se a solução contém NaNs ou infs

    if np.any(np.isnan(sol.y)) or np.any(np.isinf(sol.y)):
        print("A simulação falhou: foram encontrados NaNs ou infs nos resultados.")
        nan_inf_mask = np.isnan(sol.y) | np.isinf(sol.y)
        first_nan_inf_col = np.where(np.any(nan_inf_mask, axis=0))[0]
        if first_nan_inf_col.size > 0:
            stop_idx = first_nan_inf_col[0]
            t_sim = sol.t[:stop_idx]
            q_sim = sol.y[0:3, :stop_idx]
            dq_sim = sol.y[3:6, :stop_idx]
            print(f"Plotando resultados até o tempo {t_sim[-1]:.4f}s antes da falha.")
        else:
             print("Não foi possível determinar o ponto da falha. Plotando resultados incompletos.")
             t_sim = sol.t
             q_sim = sol.y[0:3, :]
             dq_sim = sol.y[3:6, :]
             integral_e_sim = sol.y[6:9, :]
    else:
        # Extrai os resultados da simulação
        q_sim = sol.y[0:3]  # Trajetória simulada (posição)
        dq_sim = sol.y[3:6] # Velocidade simulada
        integral_e_sim = sol.y[6:9] # Erro integral simulado
        t_sim = sol.t # Tempos da simulação

    dqd_traj_sim = np.array([np.interp(t_sim, t_des[t_des <= 31.0], qd_des[t_des <= 31.0, i]) for i in range(qd_des.shape[1])])

    # Plotando os resultados
    plt.figure(figsize=(10, 12))

    # Plot da Posição das Juntas
    plt.subplot(3, 1, 1)
    for i in range(3):
        plt.plot(t_sim, q_sim[i, :], label=f'q{i+1} Simulado', linewidth=1.5)
        # Interpolar a trajetória desejada nos tempos de simulação para plotagem
        # Ajustado para usar apenas os pontos de q_des até 31 segundos para interpolação
        qd_des_interp = np.interp(t_sim, t_des[t_des <= 31.0], q_des[t_des <= 31.0, i])
        plt.plot(t_sim, qd_des_interp, '--', label=f'q{i+1} Desejado', linewidth=1.2)
    plt.ylabel('Posição [rad ou m]')
    plt.title('Simulação do Controle PID')
    plt.legend()
    plt.grid(True)

    # Plot da Velocidade das Juntas
    plt.subplot(3, 1, 2)
    # Usar o dqd_traj_sim calculado corretamente
    for i in range(3):
        plt.plot(t_sim, dq_sim[i, :], label=f'dq{i+1} Simulado', linewidth=1.5)
        plt.plot(t_sim, dqd_traj_sim[i, :], '--', label=f'dq{i+1} Desejado', linewidth=1.2)
    plt.ylabel('Velocidade [rad/s ou m/s]')
    plt.xlabel('Tempo [s]')
    plt.legend()
    plt.grid(True)

    # Plot do Erro de Posição
    plt.subplot(3, 1, 3)
    error = np.array([np.interp(t_sim, t_des[t_des <= 31.0], q_des[t_des <= 31.0, i]) for i in range(q_des.shape[1])]) - q_sim

    for i in range(3):
        plt.plot(t_sim, error[i, :], label=f'Erro q{i+1}', linewidth=1.5)
    plt.ylabel('Erro de Posição [rad ou m]')
    plt.xlabel('Tempo [s]')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Armazenamento dos valores de posição e tempo da trajetória
    np.save('trajetoria_q_sim.npy', q_sim) 

    # Opcional: salvar também t_sim se quiser
    np.save('tempos_sim.npy', t_sim)

if __name__ == '__main__':
    run_pid_control()
