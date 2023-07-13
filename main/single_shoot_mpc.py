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
j = W.T @ I @ W;


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
u = vertcat(u1, u2, u3, u4)


# actuation dynamics
tau_beta = SX(np.array([
    [l*kf*(-u2 + u4)],
    [l*kf*(-u1 + u3)],
    [km*(-u1 + u2 - u3 + u4)]
]))
thrust = kf*(u1 + u2 + u3 + u4)


# continuous-time dynamics
Xdot = vertcat(
    x_d, y_d, z_d, phi_d, theta_d, psi_d,
    -9.81 * vertcat(0,0,1) + R @ vertcat(0,0,thrust) / m,
    inv(j) @ (tau_beta - C @ vertcat(phi_d, theta_d, psi_d))
)   
print()
print(inv(j))
print()
# time-derivative of inertial terms assumed to be 0
Xdot_aug = vertcat(Xdot, 0, 0, 0, 0, 0)



'''
MPC configuration (time horizon, time step, num control intervals?).
Define objective function.
'''
Q = SX.eye(17)                          # Cost matrices
R = SX.eye(4)
L = X_aug.T @ Q @ X_aug + u.T @ R @ u   # Objective function

T = 10      # predictive horizon length
DT = 0.1    # time step in seconds



'''
Build the integrator function.
Using RK4, integrate the dynamics and cost function.
'''
f = Function('f', [X_aug, u], [Xdot_aug, L])    # outputs continuous dynamics and objective
Xi = SX.sym('Xi', 17)   # inputted state
U = SX.sym('U', 4)    # inputted control input
Xf = Xi    # integrated dynamics 
J  = 0     # integrated objective

# RK4 definition
for i in range(4):               
    k1, k1_q = f(Xf, U)
    k2, k2_q = f(Xf + DT/2 * k1, U)
    k3, k3_q = f(Xf + DT/2 * k2, U)
    k4, k4_q = f(Xf + DT * k3, U)
    Xf += DT/6 * (k1 +2*k2 +2*k3 +k4)
    J += DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q)

# Integrator function (discrete-time dynamics/state)
F = Function('F', [Xi, U], [Xf, J],['Xi','U'],['Xf','J'])



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
Fk = F(Xi=[0,0,0,0,0,0,0,0,0,0,0,0, 
    m_val, l_val, Ixx_val, Iyy_val, Izz_val], 
    U=36*np.ones((4,1))
)
print(Fk['Xf'])
print(Fk['J'])



''' 
Formulate NLP and create solver.
x0, upper and lower bounds for states and inputs as parameters.
'''
# Start with an empty NLP
input_constr = []
lb_input = []
ub_input = []

state_constr = []
lb_state = []
ub_state = []

guess = []
J_tot = 0       # accumulated cost over the predictive horizon


# Formulate the NLP
# To achieve setpoint x*, states are written as x*-x
X_i = SX([0,0,0,0,0,0,0,0,0,0,0,0, 
    m_val, l_val, Ixx_val, Iyy_val, Izz_val
])
X_set = SX([0,0,10,0,0,0,0,0,0,0,0,0, 
    m_val, l_val, Ixx_val, Iyy_val, Izz_val
])
Xk = X_set

for k in range(T):
    # Control constraints
    Uk = SX.sym('U_' + str(k), 4)
    input_constr += [Uk]   
    '''
    lb_input += [-64, -64, -64, -64]   # lower bound on square of rotor freq
    ub_input += [64, 64, 64 ,64]    # upper bound on square of rotor freq
    guess += [SX([36,36,36,36])]
    '''

    # Integrate till the end of the interval
    Fk = F(Xi=Xk, U=Uk)
    Xk = Fk['Xf']
    J_tot = J_tot + Fk['J']
    
    # Add inequality constraint on the states
    state_constr += [Xk]
    '''
    lb_state += [
        -inf, -inf, -inf, -pi/3, -pi/3, -inf, -inf, -inf, -inf, -inf, -inf, -inf,
        -inf, -inf, -inf, -inf, -inf
    ]
    ub_state += [
        inf, inf, inf, pi/3, pi/3, inf, inf, inf, inf, inf, inf, inf,
        inf, inf, inf, inf, inf
    ]'''


# NLP notation: x--what you're solving for (control input), g--constraints
# Create an NLP solver
prob = {'f': J_tot, 'x': vertcat(*input_constr), 'g': vertcat(*state_constr)}
solver = nlpsol('solver', 'ipopt', prob)

# Solve the NLP
print(gradient(J_tot, vertcat(*input_constr)))
sol = solver()
'''
    lbx=lb_input, ubx=ub_input, lbg=lb_state, ubg=ub_state)'''
u_opt = sol['x']



''' 
All of the above steps are to initialize the MPC which is the solver.
'''

