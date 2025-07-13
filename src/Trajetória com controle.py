import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from parametros_cookbot import cookbot
from roboticstoolbox import mstraj, ERobot, Link, ET


C = np.diag([0.01, 0.1, 0.01])             # Atrito
G = np.zeros([0, 0, 9.81])                 # Gravidade (se necessário)

# Ganhos PID (editáveis facilmente)
# Reduzindo os ganhos para tentar resolver o problema de travamento na simulação
Kp = np.array([20, 40, 30])  # Ganhos proporcionais reduzidos
Kd = np.array([5, 10, 7.5])     # Ganhos derivativos reduzidos
Ki = np.array([0, 0, 0])        # Ganhos integrais (mantidos em zero por enquanto)

# IMPORTAR TRAJETÓRIA

from traj_cookbot import traj_des, q_des, qd_des, qdd_des, t_des

# Gerar dados da trajetória ao longo do tempo
# Usando os resultados de mstraj
t_span = t_des  # Usar os tempos gerados por mstraj
qd_values = q_des
dqd_values = qd_des
ddqd_values = qdd_des

def dynamics(t, y, t_des, q_des, qd_des, qdd_des, Kp, Kd, Ki):
    """
    Função que define a dinâmica do robô e o controlador PID.
    Projetada para ser usada com um resolvedor de EDOs como solve_ivp.

    Args:
        t (float): Tempo atual da simulação.
        y (np.array): Vetor de estado atual [q, dq, integral_e].
                      q: Posição das juntas (3 elementos).
                      dq: Velocidade das juntas (3 elementos).
                      integral_e: Erro integral de posição (3 elementos).
        t_des (np.array): Vetor de tempos da trajetória desejada.
        q_des (np.array): Array de posições desejadas das juntas ao longo do tempo.
        qd_des (np.array): Array de velocidades desejadas das juntas ao longo do tempo.
        qdd_des (np.array): Array de acelerações desejadas das juntas ao longo do tempo (não usado diretamente no PID clássico, mas passado por compatibilidade ou para feedforward futuro).
        Kp (np.array): Ganhos proporcionais do controlador PID.
        Kd (np.array): Ganhos derivativos do controlador PID.
        Ki (np.array): Ganhos integrais do controlador PID.

    Returns:
        np.array: Derivadas dos estados [dq, ddq, e].
    """
    # Extrai os estados atuais - MOVIDO PARA O INÍCIO
    q = y[0:3]
    dq = y[3:6]
    integral_e = y[6:9] # O erro integral é agora parte do estado


    # C e G podem ser definidos fora da função dynamics se forem constantes
    # Mas mantidos aqui para clareza no contexto da dinâmica simplificada
    # AGORA UTILIZANDO cookbot.coriolis QUE DEPENDE DE q E dq
    C = cookbot.coriolis(q, dq)  # Atrito
    G = np.zeros(3)  # Gravidade (considerada zero neste caso)


    # Encontra os valores desejados no tempo t usando interpolação
    # np.interp requer que t_des seja sorted e unique, o que mstraj fornece.
    # Adicionando verificação de limites para garantir que t esteja dentro do intervalo de t_des
    t_clamped = np.clip(t, t_des[0], t_des[-1])

    qd_t = np.array([np.interp(t_clamped, t_des, q_des[:, i]) for i in range(q_des.shape[1])])
    dqd_t = np.array([np.interp(t_clamped, t_des, qd_des[:, i]) for i in range(qd_des.shape[1])])
    # ddqd_t = np.array([np.interp(t_clamped, t_des, qdd_des[:, i]) for i in range(qdd_des.shape[1])]) # Não usado no PID básico

    # Calcula os erros
    e = qd_t - q
    de = dqd_t - dq

    # Calcula o torque de controle PID
    tau = Kp * e + Kd * de + Ki * integral_e

    # Dinâmica do robô
    # Calcule a matriz de inércia M com base na configuração atual q
    # cookbot deve estar acessível no escopo global
    M = cookbot.inertia(q) # Calcule M usando a configuração atual (q é 1D)

    # --- Debugging Prints ---
    print(f"Time: {t:.4f}")
    print(f"q: {q}")
    print(f"dq: {dq}")
    print(f"e: {e}")
    print(f"de: {de}")
    print(f"integral_e: {integral_e}")
    print(f"tau: {tau}")
    print(f"M: {M}")
    print(f"C: {C}") # Added print for Coriolis matrix
    print(f"tau - C @ dq - G: {tau - C @ dq - G}")
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


    # Calcula a aceleração das juntas
    # M * ddq = tau - C @ dq - G
    # Usando np.linalg.solve para maior estabilidade numérica
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
    # Condições iniciais: [q_inicial, dq_inicial, integral_e_inicial]
    # q_inicial = posição inicial (primeiro ponto da trajetória desejada)
    # dq_inicial = velocidade inicial (geralmente zero)
    # integral_e_inicial = erro integral inicial (geralmente zero)
    # A trajetória desejada começa em q_des[0, :], então a posição inicial ideal é essa.
    # Para fins de demonstração, vamos começar em zeros e ver o controlador corrigir.
    #y0 = np.zeros(9)  # Estado anterior: [q, dq, integral_e] começando em zero

    # Novo estado inicial: [q_des[0, :], zeros(3), zeros(3)]
    q0 = q_des[0, :]  # Posição inicial igual ao primeiro ponto da trajetória desejada
    dq0 = np.zeros(3) # Velocidade inicial zero
    integral_e0 = np.zeros(3) # Erro integral inicial zero
    y0 = np.concatenate([q0, dq0, integral_e0])


    # Vetor de tempo para a simulação (usar os tempos da trajetória desejada ou um intervalo similar)
    # Usando t_des diretamente para que os pontos de tempo da simulação correspondam aos da trajetória desejada,
    # o que simplifica a comparação e o lookup (embora interpolação seja mais geral).
    # Se quisermos simular em um intervalo de tempo diferente ou mais denso, usaríamos np.linspace
    # dentro do span de t_des. Vamos usar t_des por simplicidade agora.
    # MODIFICADO: Limitar a simulação até 31 segundos
    t_span_sim = [t_des[0], 31.0] # Intervalo de tempo para solve_ivp
    # MODIFICADO: Incluir apenas os pontos de t_des até 31 segundos
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
        # Opcional: plotar os resultados parciais até o ponto da falha
        # Encontre o primeiro ponto onde NaN/inf apareceu
        nan_inf_mask = np.isnan(sol.y) | np.isinf(sol.y)
        first_nan_inf_col = np.where(np.any(nan_inf_mask, axis=0))[0]
        if first_nan_inf_col.size > 0:
            stop_idx = first_nan_inf_col[0]
            t_sim = sol.t[:stop_idx]
            q_sim = sol.y[0:3, :stop_idx]
            dq_sim = sol.y[3:6, :stop_idx]
            integral_e_sim = sol.y[6:9, :stop_idx]
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


    # Comparando com a trajetória desejada
    # Como usamos t_des para t_eval_sim, podemos usar q_des diretamente
    # Se t_eval_sim fosse diferente, precisaríamos interpolar q_des em t_sim
    # Transponha q_des para ter juntas nas linhas e tempo nas colunas para facilitar a comparação
    # Ajustado para usar apenas os pontos de q_des até 31 segundos para comparação
    qd_traj = q_des[t_des <= 31.0, :].T


    # Calcular a velocidade desejada interpolada nos tempos de simulação
    # Corrigido para interpolar para todos os tempos t_sim para cada junta
    # Use t_sim para interpolação, não t_des
    # Ajustado para usar apenas os pontos de qd_des até 31 segundos para interpolação
    dqd_traj_sim = np.array([np.interp(t_sim, t_des[t_des <= 31.0], qd_des[t_des <= 31.0, i]) for i in range(qd_des.shape[1])])
    # dqd_traj_sim agora tem forma (num_juntas, num_tempos_simulacao)


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
    # O erro é calculado como qd_traj - q_sim.
    # qd_traj tem forma (num_juntas, num_tempos_desejados)
    # q_sim tem forma (num_juntas, num_tempos_simulacao)
    # Como t_eval_sim = t_des, num_tempos_simulacao = num_tempos_desejados.
    # Então a subtração direta funciona.
    # Calcule o erro usando a posição desejada interpolada
    # Ajustado para usar apenas os pontos de q_des até 31 segundos para o cálculo do erro
    error = np.array([np.interp(t_sim, t_des[t_des <= 31.0], q_des[t_des <= 31.0, i]) for i in range(q_des.shape[1])]) - q_sim

    for i in range(3):
        plt.plot(t_sim, error[i, :], label=f'Erro q{i+1}', linewidth=1.5)
    plt.ylabel('Erro de Posição [rad ou m]')
    plt.xlabel('Tempo [s]')
    plt.legend()
    plt.grid(True)


    plt.tight_layout()
    plt.show()

    # No final da sua função ou script após rodar a simulação:
    np.save('trajetoria_q_sim.npy', q_sim)  # Salva em arquivo binário numpy

    # Opcional: salvar também t_sim se quiser
    np.save('tempos_sim.npy', t_sim)




if __name__ == '__main__':
    run_pid_control()
