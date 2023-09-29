from sympy import init_printing
from sympy import *
import numpy as np

# Defining Operating Points in Cruise (cruise_operatingpoints_working.ipynb)
# Based on 20x8 and CREATeV performance (cd0 = 0.018, k = 0.03)
V_op = np.array([ 8.        ,  8.93157895,  9.86315789, 10.79473684, 11.72631579,
       12.65789474, 13.58947368, 14.52105263, 15.45263158, 16.38421053,
       17.31578947, 18.24736842, 19.17894737, 20.11052632, 21.04210526,
       21.97368421, 22.90526316, 23.83684211, 24.76842105, 25.7       ])

T_op = np.array([ 6.7649253 ,  6.20478542,  5.95954902,  5.94102069,  6.09457666,
        6.38492056,  6.78842218,  7.2887787 ,  7.87444802,  8.53707301,
        9.27048221, 10.07003811, 10.9322014 , 11.85423301, 12.83398647,
       13.86976042, 14.96019227, 16.1041804 , 17.30082638, 18.54939157])

n_op = np.array([ 45.05426703,  46.24468374,  48.17515383,  50.62833559,
        53.45297282,  56.54377208,  59.82721058,  63.25169271,
        66.78071971,  70.38821297,  74.05532937,  77.76829292,
        81.51691216,  85.29356104,  89.09246037,  92.90919766,
        96.74035693, 100.5832731 , 104.43584719, 108.29641212])

# Defining static conditions for cruise
[n_0, V_0, rho_0, C_D_0, X_T_0]= symbols('n_0 V_0 \\rho_0 C_{D_0} X_{T_0}')
rho_0 = 1.225

# Defining complete input variables
[V_T, V,  U_dot, W_dot, h_dot, theta, n, rho, D, m, g, S, P, T] = symbols('V_T V \dot{U} \dot{W} \\dot{h} \\theta n \\rho D m g S P T')

# Defining outputs
[C_D, C_T, J, alpha] = symbols('C_D C_T J \\alpha')

# Total Drag
C_D_total = (1/2 * rho * V_T**2 * S)**-1 * (-((U_dot+W_dot*alpha)*m) - (m * g * (-h_dot/V_T)) + X_T_0)
C_D_total

# Total Lift
C_L_total = (1/2 * rho * V_T**2 * S)**-1 * ( m* (U_dot*alpha - W_dot) + (m*g) - X_T_0*alpha )
C_L_total

# Sub Equation 1 (AOA)
alpha = asin(-h_dot/V_T) + theta

rho = P * (287 * (T+273.15))**-1 
V_T = V * sqrt(1.225) * sqrt(rho)**-1
# Subsub Equation 2 (Advance ratio)
J = V_T / (n * D)
# Sub Equation 3 (Thrust coefficient)
C_T_20x8 = (0.1298*J**3) - 0.2679*J**2 - 0.02553*J + 0.07525     # for 20 x 8
X_T_20x8 = C_T_20x8 * rho * n**2 * D**4
# Primary equation
C_D = (0.5 * rho * V_T**2 * S)**-1 * (-(U_dot*m + W_dot*m*alpha) + (m * g * (-h_dot/V_T)) + (C_T_20x8 * rho * n**2 * D**4))
C_L = (0.5 * rho * V_T**2 * S)**-1 * ((U_dot*m*alpha - W_dot*m) + (m * g * cos(-h_dot/V_T)) - (C_T_20x8 * rho * n**2 * D**4)*alpha)
# D = C_D * 0.5 * rho * V**2 * S

# Primary equation local operating point
# C_D_0 = (0.5 * rho * V**2 * S)**-1 * (-(U_dot*m + W_dot*m*alpha) + (m * g * (-h_dot/V)) + (C_T * rho * n**2 * D**4))
C_D_0 = (0.5 * rho_0 * V_0**2 * S)**-1 * X_T_0
C_L_0 = (0.5 * rho_0 * V_0**2 * S)**-1 * (m * g)

C_L_0