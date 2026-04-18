import numpy as np
import matplotlib.pyplot as plt
from Modelo import BRT
from trajetoriasRef import *

# --- PARAMETROS DE SIMULACAO ---
tms = 0.0
tempo_total = 120.0  
h = 0.05             
posicao_x = 0.0 # Inicializacao da posicao longitudinal

# Instancia o modelo
Meu_BRT = BRT()

def gerar_velocidade_longitudinal(t):
    if t <= 30.0:
        return (10.0 / 30.0) * t
    elif t <= 60.0:
        return 10.0
    else:
        return 20.0

# --- VETORES PARA ARMAZENAR DADOS ---
# --- VETORES PARA ARMAZENAR DADOS E PLOTAR ---
tempovec, y_vec, y_dot_vec, psi_vec, psi_dot_vec,x_vec = [], [], [], [], [],[]
vx_vec, delta_f_vec = [], []

# --- LOOP DE SIMULACAO ---
while tms <= tempo_total:
    #vx_atual = gerar_velocidade_longitudinal(tms)
    vx_atual = 5
    #delta_f_atual = np.deg2rad(referencia_cenario_a(tms))
    delta_f_atual = np.deg2rad(referencia_cenario_b(tms))
    
    # Integracao da posicao longitudinal (X) simples
    posicao_x += vx_atual * h
    
    # Execucao do passo de integracao do Modelo Lateral
    Meu_BRT.passo(vx_atual, delta_f_atual, h)

    # 3. Armazenar variaveis de estado e entradas
    tempovec.append(tms)
    y_vec.append(Meu_BRT.y)
    x_vec.append(posicao_x)
    y_dot_vec.append(Meu_BRT.y_dot)
    
    # Convertendo os angulos de radianos para graus visando melhor visualizacao
    psi_vec.append(np.rad2deg(Meu_BRT.psi))
    psi_dot_vec.append(np.rad2deg(Meu_BRT.psi_dot))
    
    vx_vec.append(vx_atual)
    delta_f_vec.append(np.rad2deg(delta_f_atual))
    
    tms += h

# -------------------------------------------------------------------------------------------
# ===================== PLOTS ===============================================================
# -------------------------------------------------------------------------------------------

plt.figure(figsize=(12, 8))

# Plot 1: Erro Lateral / Deslocamento
plt.subplot(2, 2, 1)
plt.plot(tempovec, y_vec, label='Erro Lateral y(t)', color='blue', linewidth=2)
plt.plot(tempovec, y_dot_vec, label='Velocidade Lateral y_dot(t)', color='turquoise', linestyle='--')
plt.title('Deslocamento Lateral do BRT')
plt.xlabel('Tempo [s]')
plt.ylabel('[m] / [m/s]')
plt.grid(True)
plt.legend()

# Plot 2: Angulo e Taxa de Guinada
plt.subplot(2, 2, 2)
plt.plot(tempovec, psi_vec, label='Ang. Guinada psi(t)', color='purple', linewidth=2)
plt.plot(tempovec, psi_dot_vec, label='Taxa Guinada psi_dot(t)', color='magenta', linestyle='--')
plt.title('Dinamica de Rotacao (Guinada)')
plt.xlabel('Tempo [s]')
plt.ylabel('[Graus] / [Graus/s]')
plt.grid(True)
plt.legend()

# Plot 3: Velocidade Longitudinal
plt.subplot(2, 2, 3)
plt.plot(tempovec, vx_vec, label='Velocidade Longitudinal vx(t)', color='green', linewidth=2)
plt.title('Perfil de Velocidade')
plt.xlabel('Tempo [s]')
plt.ylabel('Velocidade [m/s]')
plt.grid(True)
plt.legend()

# Plot 4: Entrada de Direcao
plt.subplot(2, 2, 4)
plt.plot(tempovec, delta_f_vec, label='Guinada Volante delta_f(t)', color='red', linewidth=2)
plt.title('Sinal de Entrada de Direcao (Malha Aberta)')
plt.xlabel('Tempo [s]')
plt.ylabel('Graus [deg]')
plt.grid(True)
plt.legend()

plt.figure(figsize=(12, 8))

# Plot 1: Trajetoria no Plano (X vs Y)
plt.plot(x_vec, y_vec, color='black', label='Trajetoria do BRT')
plt.title('Posicao do BRT no Plano (Trajetoria)')
plt.xlabel('Posicao Longitudinal X [m]')
plt.ylabel('Deslocamento Lateral Y [m]')
plt.grid(True, linestyle='--')
plt.legend()

plt.tight_layout()
plt.show()