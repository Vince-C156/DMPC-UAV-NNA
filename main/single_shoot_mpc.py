from casadi import *
import numpy as np
import time



''' 
Declare model variables (including parameters the RL will learn).
Derive nonlinear dynamics, concatenate them together MATLAB style.
This will be the equality constraint of the NLP.
'''
# State Variables: position, rotation, and their time-derivatives
x = SX.sym('x')
y = SX.sym('y')
z = SX.sym('z')
phi = SX.sym('phi')     # roll
theta = SX.sym('theta') # pitch
psi = SX.sym('psi')     # yaw
x_d = SX.sym('x_d')     # time-derivatives
y_d = SX.sym('y_d')
z_d = SX.sym('z_d')
phi_d = SX.sym('phi_d')
theta_d = SX.sym('theta_d')
psi_d = SX.sym('psi_d')
# state
X = vertcat(x, y, z, phi, theta, psi,\
    x_d, y_d, z_d, phi_d, theta_d, psi_d)   


# Inertial Parameters
m = SX.sym('m')     # total mass of quadcopter
l = SX.sym('l')     # dist between rotor and COM-THIS ASSUMES SYMMETRY
Ixx = SX.sym('Ixx') # moment of inertia about x-axis
Iyy = SX.sym('Iyy') # moment of inertia about y-axis
Izz = SX.sym('Izz') # moment of interia about z-axis
# Augment the state
X_aug = vertcat(X, m, l, Ixx, Iyy, Izz)


# Aerodynamic Parameters, using values from MIT paper
#kf = MX.sym('k')     # lift constant for force
#km = MX.sym('b')     # drag constant for moments
#Ax = MX.sym('Ax')    # air resistance coeff, drag force proportional to velocity
#Ay = MX.sym('Ay')    # air resistance coeff in y direction
#Az = MX.sym('Az')    # air resistance coeff in z direction
kf = 0.005022
km = 1.858 * 10**(-5)
Ax = 0
Ay = 0
Az = 0


# rotation matrix from body frame to inertial frame
Rx = SX(np.array([
    [1,           0,            0],
    [0,    cos(phi),    -sin(phi)],
    [0,    sin(phi),     cos(phi)]
]))
Ry = SX(np.array([
    [cos(theta),   0,  sin(theta)],
    [0,            1,           0],
    [-sin(theta),  0,  cos(theta)]
]))
Rz = SX(np.array([
    [cos(psi),    -sin(psi),    0],
    [sin(psi),     cos(psi),    0],
    [0,            0,           1]
]))
R = Rz @ Ry @ Rx


# calculation of jacobian matrix that converts body frame vels to inertial frame
W = SX(np.array([ 
    [1,  0,        -sin(theta)],
    [0,  cos(phi),  cos(theta)*sin(phi)],   
    [0, -sin(phi),  cos(theta)*cos(phi)]
]))
I = SX(np.array([
    [Ixx, 0, 0], 
    [0, Iyy, 0], 
    [0, 0, Izz]
]))
J = W.T @ I @ W;


# Coriolis matrix for defining angular equations of motion
C11 = 0;

C12 = (Iyy-Izz)*(theta_d*cos(phi)*sin(phi) + psi_d*(sin(phi)**2)*cos(theta)) +\
    (Izz-Iyy)*psi_d*(cos(phi)**2)*cos(theta) -\
    Ixx*psi_d*cos(theta)

C13 = (Izz-Iyy)*psi_d*cos(phi)*sin(phi)*(cos(theta)**2)

C21 = (Izz-Iyy)*(theta_d*cos(phi)*sin(phi) + psi_d*(sin(phi)**2)*cos(theta)) +\
    (Iyy-Izz)*psi_d*(cos(phi)**2)*cos(theta) +\
    Ixx*psi_d*cos(theta)

C22 = (Izz-Iyy)*phi_d*cos(phi)*sin(phi)

C23 = -Ixx*psi_d*sin(theta)*cos(theta) +\
    Iyy*psi_d*(sin(phi)**2)*sin(theta)*cos(theta) +\
    Izz*psi_d*(cos(phi)**2)*sin(theta)*cos(theta)

C31 = (Iyy-Izz)*psi_d*(cos(theta)**2)*sin(phi)*cos(phi) -\
    Ixx*theta_d*cos(theta)

C32 = (Izz-Iyy)*(theta_d*cos(phi)*sin(phi)*sin(theta) + phi_d*(sin(phi)**2)*cos(theta)) +\
    (Iyy-Izz)*phi_d*(cos(phi)**2)*cos(theta) +\
    Ixx*psi_d*sin(theta)*cos(theta) -\
    Iyy*psi_d*(sin(phi)**2)*sin(theta)*cos(theta) -\
    Izz*psi_d*(cos(phi)**2)*sin(theta)*cos(theta)

C33 = (Iyy-Izz)*phi_d*cos(phi)*sin(phi)*(cos(theta)**2) -\
    Iyy*theta_d*(sin(phi)**2)*cos(theta)*sin(theta) -\
    Izz*theta_d*(cos(phi)**2)*cos(theta)*sin(theta) +\
    Ixx*theta_d*cos(theta)*sin(theta)

C = SX(np.array([
    [C11, C12, C13], 
    [C21, C22, C23], 
    [C31, C32, C33]
]))


# Control Input is square of rotor frequency
u1 = SX.sym('u1')
u2 = SX.sym('u2')
u3 = SX.sym('u3')
u4 = SX.sym('u4')
U = vertcat(u1, u2, u3, u4)


# actuation dynamics
tau_beta = SX(np.array([
    [l*kf*(-u2 + u4)],
    [l*kf*(-u1 + u3)],
    [km*(-u1 + u2 - u3 + u4)]
]))
thrust = kf*(U[0] + U[1] + U[2] + U[3])


# continuous-time dynamics
Xdot = vertcat(
    x_d, y_d, z_d, phi_d, theta_d, psi_d,
    -9.81 * vertcat(0,0,1) + R @ vertcat(0,0,thrust) / m,
    inv(J) @ (tau_beta - C @ vertcat(phi_d, theta_d, psi_d))
)   
# time-derivative of inertial terms assumed to be 0
Xdot_aug = vertcat(Xdot, 0, 0, 0, 0, 0)



'''
MPC configuration (time horizon, time step, num control intervals?).
Define objective function.
'''
Q = SX.eye(17)                          # Cost matrices
R = SX.eye(4)
L = X_aug.T @ Q @ X_aug + U.T @ R @ U   # Objective function

T = 10      # predictive horizon length
DT = 0.1    # time step in seconds



'''
Build the integrator function.
Using RK4, integrate the dynamics and cost function.
'''
f = Function('f', [X_aug, U], [Xdot_aug, L])    # outputs continuous dynamics and objective
X0 = MX.sym('X0', 17)   # inputted state
U0 = MX.sym('U0', 4)    # inputted control input
Xf = X0    # integrated dynamics 
q  = 0     # integrated objective

# RK4 definition
for i in range(4):               
    k1, k1_q = f(Xf, U0)
    k2, k2_q = f(Xf + DT/2 * k1, U0)
    k3, k3_q = f(Xf + DT/2 * k2, U0)
    k4, k4_q = f(Xf + DT * k3, U0)
    Xf += DT/6 * (k1 +2*k2 +2*k3 +k4)
    q += DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q)

# Integrator function (discrete-time dynamics/state)
F = Function('F', [X0, U0], [Xf, q],['X0','U0'],['Xf','q'])



''' 
Test dynamics
'''
# crazyflie 2.0 default inertial params:
m_val = 0.027 # kg
l_val = 0.040 # m
Ixx_val = 2.3951 * 10**(-5)
Iyy_val = 2.3951 * 10**(-5)
Izz_val = 3.2347 * 10**(-5)

# Evaluate at a test point
st = time.time()
Fk = F(X0=[0,0,0,0,0,0,0,0,0,0,0,0, 
    m_val, l_val, Ixx_val, Iyy_val, Izz_val], 
    U0=100*np.ones((4,1))
)
et = time.time()
print(Fk['Xf'])
print(Fk['q'])



''' 
Formulate NLP.
Create NLP solver.
x0, constraints, and learned parameters as inputs.
'''




''' 
All of the above steps are to initialize the MPC which is the solver.
'''

