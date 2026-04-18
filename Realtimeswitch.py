import time
import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT

# --- CONFIGURACOES INICIAIS ---
h = 0.05          
tempo_total = 500.0 # Tempo aumentado para você explorar o mapa
Meu_BRT = BRT()

# Variaveis globais de controle
volante = 0.0 
velocidade_comando = 1.0 # Começa com 1 m/s
volante_max = 20
volante_min = -20

sensibilidade_dir = 1.5  # Graus por clique
sensibilidade_vel = 0.5  # m/s por clique

# --- FUNCOES DE EVENTO ---
def ao_pressionar(event):
    global volante, velocidade_comando
    if event.key == 'left':
        volante = min(volante + sensibilidade_dir, volante_max)
    elif event.key == 'right':
        volante = max(volante - sensibilidade_dir, volante_min)
    elif event.key == 'up':
        velocidade_comando += sensibilidade_vel
    elif event.key == 'down':
        velocidade_comando = max(0, velocidade_comando - sensibilidade_vel) # impede marcha ré negativa
    elif event.key == ' ': # Barra de espaço para resetar direção
        volante = 0.0

# --- PREPARACAO DOS GRAFICOS ---
plt.ion() 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9))
fig.canvas.mpl_connect('key_press_event', ao_pressionar)

# Grafico 1: Trajetoria Global
line_traj, = ax1.plot([], [], 'b-', label='Trajetoria BRT')
ax1.set_title('CONTROLES: Setas (Esq/Dir) = Direção | Setas (Cima/Baixo) = Velocidade')
ax1.set_xlabel('X Global [m]')
ax1.set_ylabel('Y Global [m]')
ax1.axis('equal')
ax1.grid(True)

# Grafico 2: Info de Performance
ax2_twin = ax2.twinx()
line_vel, = ax2.plot([], [], 'g-', label='Velocidade [m/s]')
line_dir, = ax2_twin.plot([], [], 'r--', label='Esterçamento [°]')

ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('Velocidade (m/s)', color='g')
ax2_twin.set_ylabel('Ângulo Volante (°)', color='r')
ax2.grid(True)

plt.tight_layout()

# --- LOOP DE SIMULACAO ---
tms = 0.0
pos_x, pos_y = 0.0, 0.0
hist_x, hist_y, hist_t, hist_v, hist_d = [], [], [], [], []

print("Simulação Pronta!")
print("SETAS LATERAIS: Curva | SETAS CIMA/BAIXO: Acelera/Freia | ESPAÇO: Alinha Rodas")

while tms <= tempo_total:
    # 1. Pegar entradas atuais
    vx_atual = velocidade_comando
    delta_f_rad = np.deg2rad(volante)
    
    # 2. Passo de integração (Dinâmica Lateral)
    # Note que se vx_atual for 0, o modelo pode ter singularidades dependendo da implementação
    vx_safe = max(1, vx_atual) 
    Meu_BRT.passo(vx_safe, delta_f_rad, h)
    
    # 3. Cinemática de Rotação (Referencial Global)
    psi = Meu_BRT.psi
    vy = Meu_BRT.y_dot
    
    vx_glob = vx_atual * np.cos(psi) - vy * np.sin(psi)
    vy_glob = vx_atual * np.sin(psi) + vy * np.cos(psi)
    
    pos_x += vx_glob * h
    pos_y += vy_glob * h
    
    # 4. Históricos
    hist_t.append(tms)
    hist_x.append(pos_x)
    hist_y.append(pos_y)
    hist_v.append(vx_atual)
    hist_d.append(volante)
    
    # 5. Update Visual (Otimizado)
    if int(tms/h) % 10 == 0:
        line_traj.set_data(hist_x, hist_y)
        line_vel.set_data(hist_t, hist_v)
        line_dir.set_data(hist_t, hist_d)
        
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()
        ax2_twin.relim()
        ax2_twin.autoscale_view()
        
        plt.pause(0.0001)
    
    tms += h

plt.ioff()
plt.show()