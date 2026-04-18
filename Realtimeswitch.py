import time
import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT

# --- CONFIGURACOES INICIAIS ---
h = 0.05             
tempo_total = 240.0 # Aumentado para voce ter tempo de dirigir
Meu_BRT = BRT()

# Variavel global para o controle manual
direcao_teclado = 0.0 
sensibilidade = 1.0 # Quantos graus o volante vira a cada clique

# --- FUNCOES DE EVENTO ---
def ao_pressionar(event):
    global direcao_teclado
    if event.key == 'left':
        direcao_teclado += sensibilidade
    elif event.key == 'right':
        direcao_teclado -= sensibilidade
    elif event.key == 'up':
        # Opcional: resetar direcao
        direcao_teclado = 0.0

# --- PREPARACAO DOS GRAFICOS ---
plt.ion() 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Conectar o evento de teclado a janela
fig.canvas.mpl_connect('key_press_event', ao_pressionar)

# Grafico 1: Trajetoria
line_traj, = ax1.plot([], [], 'b-', label='Trajetoria BRT')
ax1.set_title('Use as SETAS do teclado para guiar o BRT (UP para centralizar)')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.grid(True)

# Grafico 2: Eixo Duplo
ax2_twin = ax2.twinx()
line_psi, = ax2.plot([], [], 'r-', label='Ang. Guinada psi(t)')
line_ref, = ax2_twin.plot([], [], 'g--', label='Direcao Manual delta_f(t)')

ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('Guinada [Graus]', color='r')
ax2_twin.set_ylabel('Comando Volante [Graus]', color='g')
ax2.grid(True)

plt.tight_layout()

# --- LOOP DE SIMULACAO ---
tms = 0.0
posicao_x = 0.0
historico_x, historico_y, historico_t, historico_psi, historico_ref = [], [], [], [], []

print("Simulacao Iniciada. Clique na janela do grafico e use as setas ESQUERDA/DIREITA.")

while tms <= tempo_total:
    vx_atual = 10.0 # Velocidade constante para facilitar o controle
    
    # O comando vem da variavel global alterada pelo teclado
    delta_f_atual_deg = direcao_teclado
    delta_f_rad = np.deg2rad(delta_f_atual_deg)
    
    # Passo de integracao
    Meu_BRT.passo(vx_atual, delta_f_rad, h)
    posicao_x += vx_atual * h
    
    # Armazenar dados
    historico_t.append(tms)
    historico_x.append(posicao_x)
    historico_y.append(Meu_BRT.y)
    historico_psi.append(np.rad2deg(Meu_BRT.psi))
    historico_ref.append(delta_f_atual_deg)
    
    # Atualizar Graficos
    if int(tms/h) % 5 == 0: # Atualizacao mais frequente para resposta rapida
        line_traj.set_data(historico_x, historico_y)
        line_psi.set_data(historico_t, historico_psi)
        line_ref.set_data(historico_t, historico_ref)
        
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax2_twin.relim()
        ax2_twin.autoscale_view()
        
        plt.pause(0.001)
    
    tms += h

plt.ioff()
plt.show()