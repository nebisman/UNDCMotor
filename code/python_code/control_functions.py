import control as ct
from dcmotor_iot import MotorSystemIoT, set_controller, set_reference, step_closed, set_pid
import matplotlib
import numpy as np
from time import sleep
matplotlib.use("TkAgg", force=True)


#
"""
En este codigo tenemos la base de un controlador de dos parametros

h = 0.75
plant = SystemIoT()
G, delay, uN = plant.transfer_function(40)
num, den = ct.tfdata(G)
N0 = num[0][0][0]
N1 = 0
D1, D0 = den[0][0][0], den[0][0][1]

s = ct.TransferFunction.s
z = 1
wn = 0.12
T = wn**2/(s**2 + 2*z*wn*s + wn**2)
T0 = T/N0
Np = T0.num[0][0][0]
Dp = T0.den[0][0][0] * s**2 + T0.den[0][0][1] *s + T0.den[0][0][2]
Dpb = 1
L = Np*Dpb
F = Dp*Dpb

F1 = F.num[0][0]

F1 = np.array(F1)

F1 = np.flip(F1)

SM = np.array([[N0, 0, 0],[N1, D0, N0], [0,  D1, N1]])
X = np.linalg.solve(SM, F1)
A = X[1]*s
M = X[2]*s + X[0]
D = D1*s + D0
N = N0


A1 = [X[1], 0]
M1 = [-X[2],  -X[0]]
L1 = [L]
# print(L1)
# print(M1)
# print(A1)
num = [[L1, M1]]
den = [[A1, A1]]
C = ct.tf(num, den)
print(C.den[0])
Cve = ct.tf2ss(C)
Cd = ct.c2d(Cve, h, method ='tustin')
print(C)
set_controller(plant, C)
signal = [30,  40, 50, 60, 70, 80]
set_pid(plant, kp = 16.796, ki = 5, kd = 16.441, N = 27.38, beta = 1)

#stair_closed(plant, signal, 50)
#r, y, u = step_closed(plant, 50, 60,100,100)

 controlador """

#
kp = 0.0246048
ki = 0.02534242208361848115
kd  =0.00010707
N = 101.9052
beta = 1
# num_pid = [ kp+kd*N, ki+kp*N, ki*N]
# den_pid = [1, N, 0]
# C = ct.tf(num_pid, den_pid)
# print(C)
#
# # sistema IoT"""
#
motor1 = MotorSystemIoT()
#
# """ comandos de experimentación"""
#

