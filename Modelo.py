import numpy as np

class BRT:    
    def __init__(self,
                 cf=133e3,       # Coeficiente de rigidez pneus dianteiros [N/rad]
                 cr=157e3,       # Coeficiente de rigidez pneus traseiros [N/rad]
                 iz=118649.0,    # Momento de inercia de rotacao [kg*m^2]
                 lf=6.25,        # Distancia do CG ao eixo dianteiro [m]
                 lr=7.05,        # Distancia do CG ao eixo traseiro [m]
                 m=26000.0):     # Massa total do veiculo (com passageiros) [kg]

        # Parametros construtivos do Modelo Bicicleta
        self.Cf = cf
        self.Cr = cr
        self.Iz = iz
        self.lf = lf
        self.lr = lr
        self.m  = m

        # Estados atuais do sistema
        self.y        = 0.0      # Erro lateral (distancia do centro da pista)
        self.y_dot    = 0.0      # Velocidade lateral
        self.psi      = 0.0      # Angulo de guinada
        self.psi_dot  = 0.0      # Taxa de guinada
        
        # Variaveis de entrada 
        self.vx       = 0.0      # Velocidade longitudinal
        self.delta_f  = 0.0      # Guinada total (Motorista + ADAS)

    def f1_dy_dt(self, y_dot):
        # Derivada da posicao lateral e a velocidade lateral
        return y_dot

    def f2_dy_dot_dt(self, y_dot, psi_dot, vx, delta_f):
        # Equacao 3 do roteiro experimental
        if vx < 0.1: vx = 0.1  # Protecao contra divisao por zero
        
        termo1 = -((self.Cf + self.Cr) / (self.m * vx)) * y_dot
        termo2 = (vx - ((self.Cf * self.lf - self.Cr * self.lr) / (self.m * vx))) * psi_dot
        termo3 = (self.Cf / self.m) * delta_f
        
        return termo1 + termo2 + termo3

    def f3_dpsi_dt(self, psi_dot):
        # Derivada do angulo de guinada e a taxa de guinada
        return psi_dot

    def f4_dpsi_dot_dt(self, y_dot, psi_dot, vx, delta_f):
        # Equacao 1 do roteiro experimental
        if vx < 0.1: vx = 0.1  # Protecao contra divisao por zero
        
        termo1 = -((self.Cf * self.lf - self.Cr * self.lr) / (self.Iz * vx)) * y_dot
        termo2 = -((self.Cf * self.lf**2 + self.Cr * self.lr**2) / (self.Iz * vx)) * psi_dot
        termo3 = ((self.Cf * self.lf) / self.Iz) * delta_f
        
        return termo1 + termo2 + termo3

    def passo(self, vx_input, delta_f_input, passo_rk4):
        h = passo_rk4
        
        # Atualizacao das variaveis de entrada
        self.vx = vx_input
        self.delta_f = delta_f_input

        # Metodo de Runge-Kutta 4a Ordem (RK4) para 4 Estados
        # K1
        k1_y       = h * self.f1_dy_dt(self.y_dot)
        k1_ydot    = h * self.f2_dy_dot_dt(self.y_dot, self.psi_dot, self.vx, self.delta_f)
        k1_psi     = h * self.f3_dpsi_dt(self.psi_dot)
        k1_psidot  = h * self.f4_dpsi_dot_dt(self.y_dot, self.psi_dot, self.vx, self.delta_f)

        # K2
        k2_y       = h * self.f1_dy_dt(self.y_dot + 0.5*k1_ydot)
        k2_ydot    = h * self.f2_dy_dot_dt(self.y_dot + 0.5*k1_ydot, self.psi_dot + 0.5*k1_psidot, self.vx, self.delta_f)
        k2_psi     = h * self.f3_dpsi_dt(self.psi_dot + 0.5*k1_psidot)
        k2_psidot  = h * self.f4_dpsi_dot_dt(self.y_dot + 0.5*k1_ydot, self.psi_dot + 0.5*k1_psidot, self.vx, self.delta_f)

        # K3
        k3_y       = h * self.f1_dy_dt(self.y_dot + 0.5*k2_ydot)
        k3_ydot    = h * self.f2_dy_dot_dt(self.y_dot + 0.5*k2_ydot, self.psi_dot + 0.5*k2_psidot, self.vx, self.delta_f)
        k3_psi     = h * self.f3_dpsi_dt(self.psi_dot + 0.5*k2_psidot)
        k3_psidot  = h * self.f4_dpsi_dot_dt(self.y_dot + 0.5*k2_ydot, self.psi_dot + 0.5*k2_psidot, self.vx, self.delta_f)

        # K4
        k4_y       = h * self.f1_dy_dt(self.y_dot + k3_ydot)
        k4_ydot    = h * self.f2_dy_dot_dt(self.y_dot + k3_ydot, self.psi_dot + k3_psidot, self.vx, self.delta_f)
        k4_psi     = h * self.f3_dpsi_dt(self.psi_dot + k3_psidot)
        k4_psidot  = h * self.f4_dpsi_dot_dt(self.y_dot + k3_ydot, self.psi_dot + k3_psidot, self.vx, self.delta_f)

        # Integracao e Atualizacao Final dos Estados
        self.y       += (k1_y + 2*k2_y + 2*k3_y + k4_y) / 6.0
        self.y_dot   += (k1_ydot + 2*k2_ydot + 2*k3_ydot + k4_ydot) / 6.0
        self.psi     += (k1_psi + 2*k2_psi + 2*k3_psi + k4_psi) / 6.0
        self.psi_dot += (k1_psidot + 2*k2_psidot + 2*k3_psidot + k4_psidot) / 6.0

        # Retorna o erro lateral e o angulo de guinada para controle
        return (self.y, self.psi)