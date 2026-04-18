import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT
from trajetoriasRef import *

# --- PARAMETROS DE SIMULACAO ---
tms = 0.0
tempo_total = 120.0  
h = 0.05             

# Inicialização das coordenadas no referencial GLOBAL (Fixo no solo)
posicao_x_global = 0.0
posicao_y_global = 0.0

# Instancia o modelo lateral
Meu_BRT = BRT()

# --- VETORES PARA ARMAZENAR DADOS ---
tempovec = []
y_local_vec, y_dot_local_vec = [], []
psi_vec, psi_dot_vec = [], []
x_global_vec, y_global_vec = [], []
vx_vec, delta_f_vec = [] , []

# --- LOOP DE SIMULACAO ---
while tms <= tempo_total:
    # 1. Definir Entradas
    vx_atual = 5.0 # Velocidade longitudinal [m/s]
    delta_f_atual = np.deg2rad(referencia_cenario_a(tms)) # Ângulo de esterçamento [rad]
    
    # 2. Execução do passo de integração do Modelo Lateral (Dinâmica do Veículo)
    # Este passo calcula as variáveis de estado: y, y_dot, psi, psi_dot
    Meu_BRT.passo(vx_atual, delta_f_atual, h)

    # 3. CINEMÁTICA: Transformação para o Referencial Global (Inercial)
    # Utilizamos o ângulo de guinada (psi) para rotacionar as velocidades locais
    psi_atual = Meu_BRT.psi
    vy_local = Meu_BRT.y_dot
    
    # Projeção das velocidades locais no plano X-Y global
    # VX_global = vx*cos(psi) - vy*sin(psi)
    # VY_global = vx*sin(psi) + vy*cos(psi)
    vx_global = vx_atual * np.cos(psi_atual) - vy_local * np.sin(psi_atual)
    vy_global = vx_atual * np.sin(psi_atual) + vy_local * np.cos(psi_atual)
    
    # Integração das posições globais (Método de Euler)
    posicao_x_global += vx_global * h
    posicao_y_global += vy_global * h

    # 4. Armazenar variáveis para análise
    tempovec.append(tms)
    x_global_vec.append(posicao_x_global)
    y_global_vec.append(posicao_y_global)
    
    y_local_vec.append(Meu_BRT.y)
    y_dot_local_vec.append(Meu_BRT.y_dot)
    
    # Convertendo radianos para graus para os gráficos de estado
    psi_vec.append(np.rad2deg(Meu_BRT.psi))
    psi_dot_vec.append(np.rad2deg(Meu_BRT.psi_dot))
    
    vx_vec.append(vx_atual)
    delta_f_vec.append(np.rad2deg(delta_f_atual))
    
    tms += h

# -------------------------------------------------------------------------------------------
# ===================== PLOTS DE ESTADO (DINÂMICA) =========================================
# -------------------------------------------------------------------------------------------

plt.figure(figsize=(12, 8))

# Subplot 1: Erro Lateral Local
plt.subplot(2, 2, 1)
plt.plot(tempovec, y_local_vec, label='Desloc. Lateral Local y(t)', color='blue')
plt.title('Dinâmica Lateral Relativa')
plt.xlabel('Tempo [s]')
plt.ylabel('[m]')
plt.grid(True)
plt.legend()

# Subplot 2: Ângulo de Guinada
plt.subplot(2, 2, 2)
plt.plot(tempovec, psi_vec, label='$\psi$ (Guinada)', color='purple')
plt.title('Ângulo de Orientação (Yaw)')
plt.xlabel('Tempo [s]')
plt.ylabel('[Graus]')
plt.grid(True)
plt.legend()

# Subplot 3: Velocidade
plt.subplot(2, 2, 3)
plt.plot(tempovec, vx_vec, label='$V_x$ Longitudinal', color='green')
plt.title('Velocidade do BRT')
plt.xlabel('Tempo [s]')
plt.ylabel('[m/s]')
plt.grid(True)
plt.legend()

# Subplot 4: Entrada de Esterçamento
plt.subplot(2, 2, 4)
plt.plot(tempovec, delta_f_vec, label='$\delta_f$ (Volante)', color='red')
plt.title('Comando de Direção')
plt.xlabel('Tempo [s]')
plt.ylabel('[Graus]')
plt.grid(True)
plt.legend()

plt.tight_layout()

# -------------------------------------------------------------------------------------------
# ===================== PLOT DA TRAJETÓRIA GLOBAL (CARTESIANO) ==============================
# -------------------------------------------------------------------------------------------

plt.figure(figsize=(10, 6))
plt.plot(x_global_vec, y_global_vec, color='black', linewidth=2, label='Rastro do Veículo')
plt.title('Trajetória do BRT no Referencial Fixo (Plano XY)')
plt.xlabel('X Global [m]')
plt.ylabel('Y Global [m]')
plt.axis('equal') # Mantém a proporção 1:1 entre os eixos
plt.grid(True, linestyle='--')
plt.legend()

plt.show()