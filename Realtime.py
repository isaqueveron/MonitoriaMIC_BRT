import time
import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT
from trajetoriasRef import *

# --- CONFIGURACOES INICIAIS ---
h = 0.05             # Passo de integração (50ms)
tempo_total = 30.0
Meu_BRT = BRT()

# --- PREPARACAO DOS GRAFICOS ---
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Grafico 1: Sistema Cartesiano (X vs Y Global)
line_traj, = ax1.plot([], [], 'b-', label='Trajetoria BRT')
ax1.set_title('Visualizacao Cartesiana Real (Referencial Fixo)')
ax1.set_xlabel('Posicao Global X [m]')
ax1.set_ylabel('Posicao Global Y [m]')
ax1.axis('equal') # CRÍTICO: Mantém a proporção real da manobra
ax1.grid(True)

# Grafico 2: Sinais Temporais com Eixo Duplo
ax2_twin = ax2.twinx()

line_psi, = ax2.plot([], [], 'r-', label='Ang. Guinada psi(t)')
line_ref, = ax2_twin.plot([], [], 'g--', label='Direcao delta_f(t)')

ax2.set_title('Evolucao Temporal: Guinada vs Direcao')
ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('Guinada [Graus]', color='r')
ax2_twin.set_ylabel('Direcao [Graus]', color='g')

lines = [line_psi, line_ref]
labels = [l.get_label() for l in lines]
ax2.legend(lines, labels, loc='upper right')
ax2.grid(True)

plt.tight_layout()

# --- VARIÁVEIS DE ESTADO E HISTÓRICO ---
tms = 0.0
posicao_x_global = 0.0
posicao_y_global = 0.0
historico_x, historico_y, historico_t, historico_psi, historico_ref = [], [], [], [], []

# --- LÓGICA DE TEMPO REAL ---
print("Iniciando simulação em tempo real...")
inicio_simulacao = time.perf_counter()

while tms <= tempo_total:
    t_iteracao_inicio = time.perf_counter() # Marca o início deste passo
    
    # 1. Cálculos do Modelo
    vx_atual = 5.0 
    delta_f_atual = np.deg2rad(referencia_cenario_a(tms))
    
    Meu_BRT.passo(vx_atual, delta_f_atual, h)

    psi_atual = Meu_BRT.psi
    vy_local = Meu_BRT.y_dot 
    
    vx_global = vx_atual * np.cos(psi_atual) - vy_local * np.sin(psi_atual)
    vy_global = vx_atual * np.sin(psi_atual) + vy_local * np.cos(psi_atual)
    
    posicao_x_global += vx_global * h
    posicao_y_global += vy_global * h
    
    # 2. Atualizar Históricos
    historico_t.append(tms)
    historico_x.append(posicao_x_global)
    historico_y.append(posicao_y_global)
    historico_psi.append(np.rad2deg(psi_atual))
    historico_ref.append(np.rad2deg(delta_f_atual))
    
    # 3. Atualizar Gráficos (a cada 5 passos para suavidade)
    if int(tms/h) % 5 == 0:
        line_traj.set_data(historico_x, historico_y)
        line_psi.set_data(historico_t, historico_psi)
        line_ref.set_data(historico_t, historico_ref)
        
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax2_twin.relim()
        ax2_twin.autoscale_view()
        
        # O plt.draw() é mais leve que o plt.pause() para atualizações rápidas
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    # --- CONTROLE DE SINCRONISMO ---
    tms += h
    
    # Calcula quanto tempo deveria ter passado desde o início
    tempo_alvo = tms 
    tempo_real_decorrido = time.perf_counter() - inicio_simulacao
    
    # Se estivermos mais rápidos que o tempo real, esperamos a diferença
    atraso = tempo_alvo - tempo_real_decorrido
    if atraso > 0:
        time.sleep(atraso)
    # Se atraso < 0, significa que o processamento está lento e o tempo real "atropelou" a simulação

print(f"Simulação finalizada em {time.perf_counter() - inicio_simulacao:.2f} segundos reais.")
plt.ioff()
plt.show()