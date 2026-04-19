import numpy as np

def trajetoria_y_referencia(x_global, cenario='a'):
    """
    Calcula o Y de referência com base na posição X global.
    Cenário A: Troca de pista simples (Lane Change).
    Cenário B: Ultrapassagem / Desvio (Double Lane Change).
    """
    # Parâmetros da manobra
    x_inicio = 30.0   # Onde a manobra começa (metros)
    comprimento = 60.0 # Distância para completar a manobra (metros)
    largura_pista = 3.5 # Deslocamento lateral (metros)
    
    # Se o veículo ainda não chegou na manobra
    if x_global < x_inicio:
        return 0.0
    
    # Se já passou da manobra
    if x_global > (x_inicio + comprimento):
        return largura_pista if cenario == 'a' else 0.0

    # Normalização de X entre 0 e 1 dentro da manobra
    x_rel = (x_global - x_inicio) / comprimento

    if cenario == 'a':
        # Troca de pista simples: Função Cosseno Suave (0 a pi)
        # Y varia de 0 a largura_pista
        return (largura_pista / 2) * (1 - np.cos(np.pi * x_rel))
    
    elif cenario == 'b':
        # Ultrapassagem: Ciclo senoidal completo (0 a 2pi)
        # O veículo vai para a outra pista e volta
        return (largura_pista / 2) * (1 - np.cos(2 * np.pi * x_rel))

    return 0.0