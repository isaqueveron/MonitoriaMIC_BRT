import numpy as np
import matplotlib.pyplot as plt

def referencia_cenario_a(t):
    """
    Troca de pista: Meio ciclo senoidal entre 30s e 90s.
    Amplitude: 10 graus.
    """
    if 30.0 <= t <= 90.0:
        # t_relativo varia de 0 a 60 dentro do intervalo
        t_rel = t - 30.0
        # Mapeia 0-60s para 0-pi radianos
        return 10.0 * np.sin((np.pi * t_rel) / 60.0)
    else:
        return 0.0

def referencia_cenario_b(t):
    """
    Ultrapassagem: Ciclo senoidal completo entre 30s e 90s.
    Amplitude: 10 graus.
    """
    if 30.0 <= t <= 90.0:
        # t_relativo varia de 0 a 60 dentro do intervalo
        t_rel = t - 30.0
        # Mapeia 0-60s para 0-2pi radianos
        return 10.0 * np.sin((2 * np.pi * t_rel) / 60.0)
    else:
        return 0.0

# --- Teste Visual das Referencias ---
t_sim = np.linspace(0, 120, 1000)
psi_a = [referencia_cenario_a(t) for t in t_sim]
psi_b = [referencia_cenario_b(t) for t in t_sim]

plt.figure(figsize=(10, 5))
plt.plot(t_sim, psi_a, label='Cenario A (Troca de Pista)', linewidth=2)
plt.plot(t_sim, psi_b, label='Cenario B (Ultrapassagem)', linewidth=2, linestyle='--')
plt.axvline(30, color='grey', linestyle=':', label='Inicio da Manobra')
plt.axvline(90, color='grey', linestyle=':', label='Fim da Manobra')
plt.title('Trajetorias de Referencia $\psi_{ref}(t)$')
plt.xlabel('Tempo [s]')
plt.ylabel('Angulo de Guinada [Graus]')
plt.grid(True)
plt.legend()
plt.show()