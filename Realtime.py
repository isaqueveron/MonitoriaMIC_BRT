import time
import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT
from trajetoriasRef import *

# --- CONFIGURACOES INICIAIS ---
h = 0.05             
tempo_total = 120.0
Meu_BRT = BRT()

# --- PREPARACAO DOS GRAFICOS (MODO INTERATIVO) ---
plt.ion() 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Grafico 1: Sistema Cartesiano (X vs Y)
line_traj, = ax1.plot([], [], 'b-', label='Trajetoria BRT')
ax1.set_title('Visualizacao Cartesiana (X vs Y)')
ax1.set_xlabel('Posicao Longitudinal X [m]')
ax1.set_ylabel('Posicao Lateral Y [m]')
ax1.grid(True)

# Grafico 2: Sinais Temporais com Eixo Duplo
ax2_twin = ax2.twinx() # Cria o segundo eixo Y

line_psi, = ax2.plot([], [], 'r-', label='Ang. Guinada psi(t)')
line_ref, = ax2_twin.plot([], [], 'g--', label='Direcao delta_f(t)')

ax2.set_title('Evolucao Temporal: Guinada vs Direcao')
ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('Guinada [Graus]', color='r')
ax2_twin.set_ylabel('Direcao [Graus]', color='g')

# Unificando as legendas dos dois eixos
lines = [line_psi, line_ref]
labels = [l.get_label() for l in lines]
ax2.legend(lines, labels, loc='upper right')

ax2.grid(True)
plt.tight_layout()

# --- LOOP DE SIMULACAO EM TEMPO REAL ---
tms = 0.0
posicao_x = 0.0
historico_x, historico_y, historico_t, historico_psi, historico_ref = [], [], [], [], []

while tms <= tempo_total:
    # Usando velocidade constante de 5 m/s conforme seu script anterior
    vx_atual = 5.0
    #delta_f_atual = np.deg2rad(referencia_cenario_a(tms))
    delta_f_atual = np.deg2rad(referencia_cenario_b(tms))
    
    # Integracao da posicao longitudinal
    posicao_x += vx_atual * h
    
    # Passo de integracao do Modelo
    # Nota: delta_f_atual entra em graus na funcao de ref, mas o modelo costuma usar radianos
    Meu_BRT.passo(vx_atual, delta_f_atual, h)
    
    # Atualizar Historicos
    historico_t.append(tms)
    historico_x.append(posicao_x)
    historico_y.append(Meu_BRT.y)
    historico_psi.append(np.rad2deg(Meu_BRT.psi))
    historico_ref.append(delta_f_atual)
    
    # Atualizar Graficos a cada 10 passos
    if int(tms/h) % 10 == 0:
        # Atualiza os dados das linhas
        line_traj.set_data(historico_x, historico_y)
        line_psi.set_data(historico_t, historico_psi)
        line_ref.set_data(historico_t, historico_ref)
        
        # Ajuste dinamico do Eixo Y no Grafico 1
        ax1.relim()
        ax1.autoscale_view(scalex=True, scaley=True)
        
        # Ajuste dinamico dos Eixos no Grafico 2
        ax2.relim()
        ax2.autoscale_view()
        ax2_twin.relim()
        ax2_twin.autoscale_view()
        
        plt.pause(0.001)
    
    tms += h

plt.ioff()
plt.show()