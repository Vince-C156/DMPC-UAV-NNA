% Notes on deriving the state space model for quadrotors
% phi: roll, theta: pitch, psi: yaw
syms x y z x_dot y_dot z_dot phi theta psi phi_dot theta_dot psi_dot;
% g makes system affine, augment state with it
syms g
% 12 states + gravity
state = [x; y; z; phi; theta; psi; 
    x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; g];



% Rotation matrix R_ZYX from body frame to inertial frame
% NECESSARY FOR LINEAR ACCELERATIONS IN STATE-DOT
Rx = [ 1,           0,          0;
       0,           cos(phi),  -sin(phi);
       0,           sin(phi),   cos(phi) ];
Ry = [ cos(theta),  0,          sin(theta);
       0,           1,          0;
      -sin(theta),  0,          cos(theta) ];
Rz = [cos(psi),    -sin(psi),   0;
      sin(psi),     cos(psi),   0;
      0,            0,          1 ];
% rotation matrix from body frame to inertial frame
R = Rz*Ry*Rx; 

disp('Rotation matrix:')
latex(R)



% Transformation matrix for angular velocities from inertial to body frame
% Necessary for the Jacobian that converts body to inertial frame velocites
W = [ 1,  0,        -sin(theta);
      0,  cos(phi),  cos(theta)*sin(phi);   
      0, -sin(phi),  cos(theta)*cos(phi) ];

syms Ixx Iyy Izz
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
% Jacobian that converts body frame velocities to inertial frame
J = W.'*I*W;

disp('Body to inertial frame velocity conversion:')
latex(J)

%ang_d_body = W * ang_d;



% Coriolis matrix for defining equations of motion
% NECESSARY FOR ANGULAR ACCELERATIONS IN STATE-DOT
C11 = 0;

C12 = (Iyy-Izz)*(theta_dot*cos(phi)*sin(phi) + psi_dot*(sin(phi)^2)*cos(theta)) +...
    (Izz-Iyy)*psi_dot*(cos(phi)^2)*cos(theta) -...
    Ixx*psi_dot*cos(theta);

C13 = (Izz-Iyy)*psi_dot*cos(phi)*sin(phi)*(cos(theta)^2);

C21 = (Izz-Iyy)*(theta_dot*cos(phi)*sin(phi) + psi_dot*(sin(phi)^2)*cos(theta)) +...
    (Iyy-Izz)*psi_dot*(cos(phi)^2)*cos(theta) +...
    Ixx*psi_dot*cos(theta);

C22 = (Izz-Iyy)*phi_dot*cos(phi)*sin(phi);

C23 = -Ixx*psi_dot*sin(theta)*cos(theta) +...
    Iyy*psi_dot*(sin(phi)^2)*sin(theta)*cos(theta) +...
    Izz*psi_dot*(cos(phi)^2)*sin(theta)*cos(theta);

C31 = (Iyy-Izz)*psi_dot*(cos(theta)^2)*sin(phi)*cos(phi) -...
    Ixx*theta_dot*cos(theta);

C32 = (Izz-Iyy)*(theta_dot*cos(phi)*sin(phi)*sin(theta) + phi_dot*(sin(phi)^2)*cos(theta)) +...
    (Iyy-Izz)*phi_dot*(cos(phi)^2)*cos(theta) +...
    Ixx*psi_dot*sin(theta)*cos(theta) -...
    Iyy*psi_dot*(sin(phi)^2)*sin(theta)*cos(theta) -...
    Izz*psi_dot*(cos(phi)^2)*sin(theta)*cos(theta);

C33 = (Iyy-Izz)*phi_dot*cos(phi)*sin(phi)*(cos(theta)^2) -...
    Iyy*theta_dot*(sin(phi)^2)*cos(theta)*sin(theta) -...
    Izz*theta_dot*(cos(phi)^2)*cos(theta)*sin(theta) +...
    Ixx*theta_dot*cos(theta)*sin(theta);

C = [C11 C12 C13; C21 C22 C23; C31 C32 C33];

disp('Coriolis matrix:')
latex(C)



% k: lift constant, m*kg
% m: total mass, kg
% b: drag constant
% l: dist between rotor and COM
% D: coeff describing overall drag due to velocity, kg/s
syms k m l b Dx Dy Dz
D = [Dx 0 0; 0 Dy 0; 0 0 Dz];
% square of motor frequency
syms u1 u2 u3 u4
control = [u1; u2; u3; u4];



% positional vars
pos = [state(1); state(2); state(3)]; 
pos_d = [state(7); state(8); state(9)];
% positional eq of motion 
pos_dd_A = [0;0;state(13)] - D*(1/m)*pos_d;
pos_dd_B = R*[0 0 0 0; 0 0 0 0; 1 1 1 1]*(k/m)*control;

% angular vars
ang = [state(4); state(5); state(6)];
ang_d = [state(10); state(11); state(12)];
% angular eq of motion
ang_dd_A = -inv(J)*C*ang_d;
ang_dd_B = inv(J)*[0 -l*k 0 l*k; -l*k 0 l*k 0; -b b -b b]*control;



%{
% linearize eqs of motion by plugging in fixed points
% roll and pitch fixed points consist of -15, 0, and 15 deg
% roll, pitch, and yaw rates linearized around 0 deg/s
roll_pitch_fixed_pts = deg2rad([-15 0 15]);
All_As = sym(zeros(3, 3, 13, 13));
All_Bs = sym(zeros(3, 3, 13, 4));

for i = 1:size(roll_pitch_fixed_pts, 2)
    for j = 1:size(roll_pitch_fixed_pts, 2)

        roll_fix = roll_pitch_fixed_pts(i);
        pitch_fix = roll_pitch_fixed_pts(j);
        
        R_lin = subs(R, ...
            {phi, theta, sin(psi), cos(psi), phi_dot, theta_dot, psi_dot},...
            {roll_fix, pitch_fix, psi, 1, 0, 0, 0});
        J_lin = subs(J, ...
            {phi, theta, sin(psi), cos(psi), phi_dot, theta_dot, psi_dot},...
            {roll_fix, pitch_fix, psi, 1, 0, 0, 0});
        C_lin = subs(C, ...
            {phi, theta, sin(psi), cos(psi), phi_dot, theta_dot, psi_dot},...
            {roll_fix, pitch_fix, psi, 1, 0, 0, 0});

        pos_dd_A_lin = [0;0;state(13)] - D*(1/m)*pos_d;
        pos_dd_B_lin =  R_lin*[0 0 0 0; 0 0 0 0; 1 1 1 1]*(k/m)*control;
        ang_dd_A_lin = -inv(J_lin)*C_lin*ang_d;
        ang_dd_B_lin = inv(J_lin)*[0 -l*k 0 l*k; -l*k 0 l*k 0; -b b -b b]*control;

        A = jacobian([pos_d; ang_d; pos_dd_A_lin; ang_dd_A_lin; 0], ...
            [pos; ang; pos_d; ang_d; state(13)]);
        All_As(i,j,:,:) = A;

        B = jacobian([pos_d; ang_d; pos_dd_B_lin; ang_dd_B_lin; 0], ...
            control);
        All_Bs(i,j,:,:) = B;

    end
end

disp('All linearized system matrices')
for i = 1:size(roll_pitch_fixed_pts, 2)
    for j = 1:size(roll_pitch_fixed_pts, 2)
        A = squeeze(All_As(i,j,:,:));
        latex(A)
    end
end

disp('All linearized input matrices')
for i = 1:size(roll_pitch_fixed_pts, 2)
    for j = 1:size(roll_pitch_fixed_pts, 2)
        B = squeeze(All_Bs(i,j,:,:));
        latex(B)
    end
end

%}



% continuous-time system matrix linearized around 0 1st-order rates.
% Augment state with g constant since system is affine.
A = jacobian( ...
    [pos_d; ang_d; pos_dd_A; ang_dd_A; 0], ...
    [pos; ang; pos_d; ang_d; state(13)] ...
);
B = jacobian( ...
    [pos_d; ang_d; pos_dd_B; ang_dd_B; 0], ...
    control ...
);



% We ony linearize for first-order rates. If the rotation matrix 
% is linearized then the drone will only fly straight up.
% Using parameter values from matlab derivation, assuming no drag
gVal=-9.81; IxxVal=1.2; IyyVal=1.2; IzzVal=2.3; 
kVal=1; mVal=2; lVal=0.25; bVal=0.2;
DxVal=0; DyVal=0; DzVal=0; 

A_hov = subs(A, ...
    {phi, theta, psi, phi_dot, theta_dot, psi_dot, state(13)},...%, Ixx, Iyy, Izz, ...
    ...   %k, m, l, b, Dx, Dy, Dz}, ...
    {0, 0, 0, 0, 0, 0, gVal}...%, IxxVal, IyyVal, IzzVal, ...
    ...   %kVal, mVal, lVal, bVal, DxVal, DyVal, DzVal} ...
);
B_hov = subs(B, ...
    {phi, theta, psi, phi_dot, theta_dot, psi_dot, state(13)},...%, Ixx, Iyy, Izz, ...
    ...   %k, m, l, b, Dx, Dy, Dz}, ...
    {0, 0, 0, 0, 0, 0, gVal}...%, IxxVal, IyyVal, IzzVal, ...
    ...   %kVal, mVal, lVal, bVal, DxVal, DyVal, DzVal} ...
);

disp('Nonlinear system and input matrices:')
latex(A)
latex(B)
disp('system and input matrices linearized around hover and 0 heading:')
latex(A_hov)
latex(B_hov)


%{
% converting continuous-time state space to discrete-time
% desired sample time
T = 0.1;
% exact discretization of linearized A
Ad_exact_lin = expm(A_lin*T);
% bilinear approximation
Ad_bi_lin = (eye(size(A_lin)) + 0.5*A_lin*T) * ...
    inv(eye(size(A_lin)) - 0.5*A_lin*T);

Ad_bi = (eye(size(A)) + 0.5*A*T) * ...
    inv(eye(size(A)) - 0.5*A*T);
Ad_bi_lin = subs(Ad_bi, ...
    {phi_dot, theta_dot, psi_dot, state(13)},...%, Ixx, Iyy, Izz, ...
    ...   %k, m, l, b, Dx, Dy, Dz}, ...
    {0, 0, 0, gVal}...%, IxxVal, IyyVal, IzzVal, ...
    ...   %kVal, mVal, lVal, bVal, DxVal, DyVal, DzVal} ...
);

disp('Linearized system matrix discretized with matrix exponential')
latex(Ad_exact_lin)
disp('Linearized system matrix discretized with bilinear transform')
latex(Ad_bi_lin)



% B is nonlinear due to not linearizing with respect to orientation
% Use bilinear approximation
Bd_bi = inv(A) * (Ad_bi - eye(size(Ad_bi))) * B;
Bd_bi_lin = subs(Bd_bi, ...
    {phi_dot, theta_dot, psi_dot, state(13)},...%, Ixx, Iyy, Izz, ...
    ...   %k, m, l, b, Dx, Dy, Dz}, ...
    {0, 0, 0, gVal}...%, IxxVal, IyyVal, IzzVal, ...
    ...   %kVal, mVal, lVal, bVal, DxVal, DyVal, DzVal} ...
);
disp('Partially linearized input matrix with bilinear transform')
latex(Bd_bi_lin)



% get discrete-time state function and write it to function
x_k1 = Ad_bi_lin*state + Bd_bi*control;
matlabFunction(x_k1,'File','quad_disc_state_fcn','Vars',{state,control});


%{
disp('eigenvalues of continuous-linear, discrete-linear, discrete-linear with bilinear transform')
latex(eig(A_lin))
latex(eig(Ad_exact_lin))
latex(eig(Ad_bi_lin))
%}

%}
