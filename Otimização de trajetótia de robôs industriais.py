import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from deap import base, creator, tools, algorithms
import random

# ===================================================
# MODELO DO ROBÔ
# G(s) = 1 / (s² + 10s + 20)
# ===================================================

num = [1]
den = [1, 10, 20]

G = ctrl.TransferFunction(num, den)

# ===================================================
# FUNÇÃO PARA CALCULAR MÉTRICAS
# ===================================================

def calcular_metricas(t, y):

    erro = 1 - y

    # Overshoot
    overshoot = max(0, (np.max(y) - 1) * 100)

    # Erro em regime permanente
    erro_regime = abs(1 - y[-1])

    # IAE
    iae = np.trapezoid(np.abs(erro), t)

    # Tempo de acomodação (±2%)
    banda = 0.02
    tempo_acomodacao = None

    for i in range(len(y)):
        if np.all(np.abs(y[i:] - 1) <= banda):
            tempo_acomodacao = t[i]
            break

    return overshoot, tempo_acomodacao, erro_regime, iae


# ===================================================
# FUNÇÃO DE AVALIAÇÃO DO AG
# ===================================================

def avaliar_pid(individuo):

    Kp, Ki, Kd = individuo

    try:

        C = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])

        sistema = ctrl.feedback(C * G, 1)

        t = np.linspace(0, 10, 1000)

        t, y = ctrl.step_response(sistema, t)

        erro = 1 - y

        iae = np.trapezoid(np.abs(erro), t)

        return (iae,)

    except:

        return (1e6,)


# ===================================================
# PID CONVENCIONAL
# ===================================================

Kp = 50
Ki = 30
Kd = 5

C = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])

# ===================================================
# SISTEMAS EM MALHA FECHADA
# ===================================================

sistema_sem_controle = ctrl.feedback(G, 1)

sistema_pid = ctrl.feedback(C * G, 1)

# ===================================================
# RESPOSTAS AO DEGRAU
# ===================================================

t = np.linspace(0, 10, 1000)

t_sc, y_sc = ctrl.step_response(sistema_sem_controle, t)

t_pid, y_pid = ctrl.step_response(sistema_pid, t)

# ===================================================
# MÉTRICAS PID CONVENCIONAL
# ===================================================

ov_pid, ts_pid, er_pid, iae_pid = calcular_metricas(
    t_pid,
    y_pid
)

# ===================================================
# RESULTADOS PID CONVENCIONAL
# ===================================================

print("\nPID CONVENCIONAL")

print(f"Kp = {Kp}")
print(f"Ki = {Ki}")
print(f"Kd = {Kd}")

print(f"Overshoot = {ov_pid:.2f}%")
print(f"Tempo de acomodação = {ts_pid:.2f} s")
print(f"Erro em regime permanente = {er_pid:.5f}")
print(f"IAE = {iae_pid:.5f}")

# ===================================================
# ALGORITMO GENÉTICO
# ===================================================

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()

toolbox.register("kp", random.uniform, 0, 150)
toolbox.register("ki", random.uniform, 0, 250)
toolbox.register("kd", random.uniform, 0, 20)

toolbox.register(
    "individual",
    tools.initCycle,
    creator.Individual,
    (toolbox.kp, toolbox.ki, toolbox.kd),
    n=1
)

toolbox.register(
    "population",
    tools.initRepeat,
    list,
    toolbox.individual
)

toolbox.register("evaluate", avaliar_pid)

toolbox.register(
    "mate",
    tools.cxBlend,
    alpha=0.5
)

toolbox.register(
    "mutate",
    tools.mutGaussian,
    mu=0,
    sigma=10,
    indpb=0.2
)

toolbox.register(
    "select",
    tools.selTournament,
    tournsize=3
)

# ===================================================
# EXECUÇÃO DO AG
# ===================================================

pop = toolbox.population(n=40)

estatisticas = tools.Statistics(
    lambda ind: ind.fitness.values
)

estatisticas.register(
    "min",
    np.min
)

pop, logbook = algorithms.eaSimple(
    pop,
    toolbox,
    cxpb=0.7,
    mutpb=0.2,
    ngen=30,
    stats=estatisticas,
    verbose=False
    
)
# Melhor indivíduo encontrado pelo AG
melhor = tools.selBest(pop, 1)[0]

Kp_ag, Ki_ag, Kd_ag = melhor
# ===================================================
# PID OTIMIZADO
# ===================================================

C_ag = ctrl.TransferFunction(
    [Kd_ag, Kp_ag, Ki_ag],
    [1, 0]
)

sistema_ag = ctrl.feedback(C_ag * G, 1)

t_ag, y_ag = ctrl.step_response(
    sistema_ag,
    t
)
# ===================================================
# ERROS DOS CONTROLADORES
# ===================================================

erro_pid = 1 - y_pid

erro_ag = 1 - y_ag

# ===================================================
# MÉTRICAS PID OTIMIZADO
# ===================================================

ov_ag, ts_ag, er_ag, iae_ag = calcular_metricas(
    t_ag,
    y_ag
)

# ===================================================
# RESULTADOS PID OTIMIZADO
# ===================================================

print("\nPID OTIMIZADO PELO AG")

print(f"Kp = {Kp_ag:.2f}")
print(f"Ki = {Ki_ag:.2f}")
print(f"Kd = {Kd_ag:.2f}")

print(f"Overshoot = {ov_ag:.2f}%")
print(f"Tempo de acomodação = {ts_ag:.2f} s")
print(f"Erro em regime permanente = {er_ag:.5f}")
print(f"IAE = {iae_ag:.5f}")

# ===================================================
# TABELA DE COMPARAÇÃO
# ===================================================

print("\n==============================")
print("COMPARAÇÃO DOS CONTROLADORES")
print("==============================")

print(f"{'Parâmetro':<25}{'PID Conv.':<15}{'PID + AG'}")

print("-"*55)

print(f"{'Overshoot (%)':<25}{ov_pid:<15.2f}{ov_ag:.2f}")
print(f"{'Tempo acomodação (s)':<25}{ts_pid:<15.2f}{ts_ag:.2f}")
print(f"{'Erro em regime':<25}{er_pid:<15.5f}{er_ag:.5f}")
print(f"{'IAE':<25}{iae_pid:<15.5f}{iae_ag:.5f}")

# ===================================================
# GRÁFICO
# ===================================================

plt.figure(figsize=(8,5))

plt.plot(
    t_sc,
    y_sc,
    linewidth=2,
    label='Sem controle'
)

plt.plot(
    t_pid,
    y_pid,
    linewidth=2,
    label='PID convencional'
)

plt.plot(
    t_ag,
    y_ag,
    linewidth=2,
    label='PID otimizado por AG'
)

plt.axhline(
    1,
    color='red',
    linestyle='--',
    label='Referência'
)

plt.xlabel('Tempo (s)')
plt.ylabel('Saída')
plt.title('Comparação entre os controladores')

plt.grid()
plt.legend()

plt.show()
# ===================================================
# FIGURA 2 - ERRO AO LONGO DO TEMPO
# ===================================================

plt.figure(figsize=(8,5))

plt.plot(
    t_pid,
    erro_pid,
    linewidth=2,
    label='PID convencional'
)

plt.plot(
    t_ag,
    erro_ag,
    linewidth=2,
    label='PID otimizado por AG'
)

plt.xlabel('Tempo (s)')
plt.ylabel('Erro')

plt.title(
    'Erro ao longo do tempo'
)

plt.grid()

plt.legend()

plt.show()
# ===================================================
# FIGURA 3 - CONVERGÊNCIA DO AG
# ===================================================

geracoes = logbook.select("gen")

melhores_iae = logbook.select("min")


plt.figure(figsize=(8,5))

plt.plot(
    geracoes,
    melhores_iae,
    marker='o'
)

plt.xlabel('Geração')

plt.ylabel('IAE')

plt.title(
    'Convergência do Algoritmo Genético'
)

plt.grid()

plt.show()