from casadi import *
from math import cos, sin



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
yaw_d = SX.sym('yaw_d')
state = vertcat(x, y, z, phi, theta, psi,\
    x_d, y_d, z_d, phi_d, theta_d, yaw_d)

# Aerodynamic Parameters
x = SX.sym('k')     # lift constant for thrust 
x = SX.sym('b')     # drag constant for torque
x = SX.sym('Ax')    # air resistance coeff, drag force proportional to velocity
x = SX.sym('Ay')    # air resistance coeff in y direction
x = SX.sym('Az')    # air resistance coeff in z direction


# Inertial Parameters
x = SX.sym('m')     # total mass of quadcopter
x = SX.sym('l')     # dist between rotor and COM-THIS ASSUMES SYMMETRY
Ixx = SX.sym('Ixx') # moment of inertia about x-axis
Iyy = SX.sym('Iyy') # moment of inertia about y-axis
Izz = SX.sym('Izz') # moment of interia about z-axis


# rotation matrix from body frame to inertial frame
Rx = MX([
    [1,           0,            0],
    [0,    cos(phi),    -sin(phi)],
    [0,    sin(phi),     cos(phi)]
])
Ry = MX([
    [cos(theta),   0,  sin(theta)],
    [0,            1,           0],
    [-sin(theta),  0,  cos(theta)]
])
Rz = MX([
    [cos(psi),    -sin(psi),    0],
    [sin(psi),     cos(psi),    0],
    [0,            0,           1]
])
R = Rz @ Ry @ Rx


# calculation of jacobian matrix that converts body frame vels to inertial frame
W = MX([ 
    [1,  0,        -sin(theta)],
    [0,  cos(phi),  cos(theta)*sin(phi)],   
    [0, -sin(phi),  cos(theta)*cos(phi)]
])
I = MX([
    [Ixx, 0, 0], 
    [0, Iyy, 0], 
    [0, 0, Izz]
])
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

C = MX([[C11, C12, C13], [C21, C22, C23], [C31, C32, C33]])



'''
MPC configuration (time horizon, time step, num control intervals?).
Define objective function.
'''



'''
Derive the integrator function.
Using RK4, integrate the dynamics and cost function.
'''



''' 
Formulate NLP.
Create NLP solver.
x0, constraints, and learned parameters as inputs.
'''



''' 
All of the above steps are to initialize the MPC which is the solver.
'''

