%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2024/01/12
%Projet for ARS5 finished by 4 students who are searching their master degrees in UTC in semester A23
%Runkun LUO, Guillaume POIDATZ, Jichuan ZHANG, Boyang MU

%Introduction of this projet:
%sujet: multi-agents(3 drones) to control
%mission 1: realisation of dynamique modele of drone
%mission 2: Calcute the trajectory of 3 drone by using consensus methode
%mission 1; using backsteping methode which takes the trajectory as un input to control the each drone
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;

%% Parameters of drone (same values for three drones)
m = 0.6150; % mass
g = 9.81; % gravity

omega = 0.4; % We define omega = -w1-w2+w3+w4, with wi the angular velocity for each rotor.
Jx = 0.0154;
Jy = 0.0154;
Jz = 0.0309;
Jr = 0.05; % Inertia of the entire motors and rotors around the axis of the body.

a1 = (Jy - Jz) / Jx;  b1 = Jr * omega / Jx;  c1 = 1 / Jx;
a2 = (Jz - Jx) / Jy;  b2 = Jr * omega / Jy;  c2 = 1 / Jy;
a3 = (Jx - Jy) / Jz;  c3 = 1 / Jz;

%% Initialization of these 3 drones
% Initial position, velocity, acceleration, and pose
x_1 = 0; 
y_1 = 0; 
z_1 = 0; 
[x_1_dot, y_1_dot, z_1_dot, phi_1, phi_1_dot, theta_1, theta_1_dot, psi_1, psi_1_dot] = deal(0);

x_2 = -10;
y_2 = 0;
z_2 = 0;
[x_2_dot, y_2_dot, z_2_dot, phi_2, phi_2_dot, theta_2, theta_2_dot, psi_2, psi_2_dot] = deal(0);

x_3 = -30;
y_3 = 0;
z_3 = 0;
[x_3_dot, y_3_dot, z_3_dot, phi_3, phi_3_dot, theta_3, theta_3_dot, psi_3, psi_3_dot] = deal(0);

% Target position, velocity, acceleration, and pose
dx12 = -20; dy12 = 20; dz12 = 0;
dx23 = 40; dy23 = 0; dz23 = 0;

xd_1 = 20;  xd_1_v = 0;  xd_1_a = 0;
yd_1 = 30;  yd_1_v = 0;  yd_1_a = 0;
zd_1 = 100;  zd_1_v = 0;  zd_1_a = 0;
phid_1 = 2;  dphid_1 = 0;  ddphid_1 = 0;
thetad_1 = 2;  dthetad_1 = 0;  ddthetad_1 = 0;
psid_1 = 1;  dpsid_1 = 0;  ddpsid_1 = 0;
 
xd_2 = xd_1 - dx12;  xd_2_v = 0;  xd_2_a = 0;
yd_2 = yd_1 - dy12;  yd_2_v = 0;  yd_2_a = 0;
zd_2 = zd_1 - dz12;  zd_2_v = 0;  zd_2_a = 0;
phid_2 = 2;  dphid_2 = 0;  ddphid_2 = 0;
thetad_2 = 2;  dthetad_2 = 0;  ddthetad_2 = 0;
psid_2 = 1;  dpsid_2 = 0;  ddpsid_2 = 0;

xd_3 = xd_2 - dx23;  xd_3_v = 0;  xd_3_a = 0;
yd_3 = yd_2 - dy23;  yd_3_v = 0;  yd_3_a = 0;
zd_3 = zd_2 - dz23;  zd_3_v = 0;  zd_3_a = 0;
phid_3 = 2;  dphid_3 = 0;  ddphid_3 = 0;
thetad_3 = 2;  dthetad_3 = 0;  ddthetad_3 = 0;
psid_3 = 1;  dpsid_3 = 0;  ddpsid_3 = 0;

%% calculate the trajectory of the center of mass
x_cm = 1/3*(x_1 + x_2 + x_3);
y_cm = 1/3*(y_1 + y_2 + y_3);
z_cm = 1/3*(z_1 + z_2 + z_3);
p_cm_initial = [x_cm, y_cm, z_cm];

% Elliptical spiral path
a_radius = 200; % The long half-axis of the elliptical trajectory
b_radius = 100; % The short half-axis of an elliptical trajectory

num_steps = 4000; % Adjusted the period to get a longer animation
dt = 0.01; % Adjusted time step for better animation

[x_cmd_array, y_cmd_array, z_cmd_array] = deal(zeros(num_steps, 1));

for i = 1:num_steps
    % traj_1
%     x_cmd_array(i) = 1/3*(xd_1 + xd_2 + xd_3); % this is the simplest tracjectory for the centre masse. only one point as the destination
%     y_cmd_array(i) = 1/3*(yd_1 + yd_2 + yd_3);
%     z_cmd_array(i) = 1/3*(zd_1 + zd_2 + zd_3);

    % traj_3
%     x_cmd_array(i) = 1/3*(xd_1 + xd_2 + xd_3) + 0.1*i*2; % 
%     y_cmd_array(i) = 1/3*(yd_1 + yd_2 + yd_3) + 0.1*i*2;
%     z_cmd_array(i) = 1/3*(zd_1 + zd_2 + zd_3) + 0.1*1*2;
    
%     if mod(i,2)==1 % after we define the next point of cm, we keep it for 4 steps in order to give drone enough time to follow this point
%         x_cmd_array(i) = x_cm - a_radius + a_radius * cos(1*i*dt);
%         y_cmd_array(i) = y_cm + b_radius * sin(1*i*dt);
%         z_cmd_array(i) = z_cm + 2*i*dt;
%     else
%         x_cmd_array(i) = x_cmd_array(i-1);
%         y_cmd_array(i) = y_cmd_array(i-1);
%         z_cmd_array(i) = z_cmd_array(i-1);
%     end

    %traj_2
    x_cmd_array(i) = x_cm - a_radius + a_radius * cos(1*i*dt);
    y_cmd_array(i) = y_cm + b_radius * sin(1*i*dt);
    z_cmd_array(i) = z_cm + 10*i*dt;
end

%% main loop

%Function "multi_agent_positions_calculs_v2" is used to calcule the respective trajectory 
%Parametres:
%dx12: the distance between drone1 and drone2 in the direction of the x-axis = x1-x2; dy23: the distance between drone2 and drone3 in the direction of the y-axis = y2-y3;
%x_cmd_array, y_cmd_array, z_cmd_array: the trajectrory designed of the center of mass
%x_1, y_1, z_1: the initial position of the drone1
[p_1, p_2, p_3, p_cm] = multi_agent_positions_calculs_v2(x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_cmd_array, y_cmd_array, z_cmd_array, dx12, dy12, dz12, dx23, dy23, dz23, num_steps, dt);


% allocate space for the arrays which stores state values(for each drone)
[phi_1_array, theta_1_array, psi_1_array, z_1_array, x_1_array, y_1_array, phi_1_dot_array, theta_1_dot_array, psi_1_dot_array, z_1_dot_array, x_1_dot_array, y_1_dot_array] = deal(zeros(num_steps, 1));
[phi_2_array, theta_2_array, psi_2_array, z_2_array, x_2_array, y_2_array, phi_2_dot_array, theta_2_dot_array, psi_2_dot_array, z_2_dot_array, x_2_dot_array, y_2_dot_array] = deal(zeros(num_steps, 1));
[phi_3_array, theta_3_array, psi_3_array, z_3_array, x_3_array, y_3_array, phi_3_dot_array, theta_3_dot_array, psi_3_dot_array, z_3_dot_array, x_3_dot_array, y_3_dot_array] = deal(zeros(num_steps, 1));

for t = 1:num_steps
    % Adding noises to states
    % The main elements of the noise problem are analysed:
    % a. The amount of noise that each state can withstand is checked by varying the coefficient in front of randn
    %    Assessment indicators: 
    %    1. Within 200 steps (the simulation time can be reduced or extended appropriately, depending on the convergence). 
    %       Whether to reach a stable value (stable value interval).
    %       Note that the stable value here is not strictly the objective value. 
    %       For example, if the target value of x is 10, since the system is not able to converge perfectly to 10, 
    %       we can consider that any value in the range of 10 +- 0.05 is considered to reach the stable value.
    %    2. If a stabilised value is reached, the change in system performance compared to the previous situation without noise. 
    %       For example, whether there is a certain delay, whether there is still a certain fluctuation in the final, etc.. 
    %    3. Compare the fluctuations of the curves to reflect the effect of noise on the system.
    % b. Extension analysis: whether the size of the noise amplitude that can be added is related to the target value of that state quantity. 
    %    For example, when zd is 100, it can withstand 10*randn noise, and when zd is 10 the original noise will significantly interfere with the system.
    
    x1_1 = phi_1 + 0.1*randn(1);
    x2_1 = phi_1_dot + 0.1*randn(1);
    x3_1 = theta_1 + 0.1*randn(1);
    x4_1 = theta_1_dot + 0.1*randn(1);
    x5_1 = psi_1 + 0.1*randn(1);
    x6_1 = psi_1_dot + 0.1*randn(1);
    x7_1 = z_1 + 0.1*randn(1);
    x8_1 = z_1_dot + 0.1*randn(1);
    x9_1 = x_1 + 0.1*randn(1);
    x10_1 = x_1_dot + 0.1*randn(1);
    x11_1 = y_1 + 0.1*randn(1);
    x12_1 = y_1_dot + 0.1*randn(1);
    
    x1_2 = phi_2 + 0.1*randn(1);
    x2_2 = phi_2_dot + 0.1*randn(1);
    x3_2 = theta_2 + 0.1*randn(1);
    x4_2 = theta_2_dot + 0.1*randn(1);
    x5_2 = psi_2 + 0.1*randn(1);
    x6_2 = psi_2_dot + 0.1*randn(1);
    x7_2 = z_2 + 0.1*randn(1);
    x8_2 = z_2_dot + 0.1*randn(1);
    x9_2 = x_2 + 0.1*randn(1);
    x10_2 = x_2_dot + 0.1*randn(1);
    x11_2 = y_2 + 0.1*randn(1);
    x12_2 = y_2_dot + 0.1*randn(1);
    
    x1_3 = phi_3 + 0.1*randn(1);
    x2_3 = phi_3_dot + 0.1*randn(1);
    x3_3 = theta_3 + 0.1*randn(1);
    x4_3 = theta_3_dot + 0.1*randn(1);
    x5_3 = psi_3 + 0.1*randn(1);
    x6_3 = psi_3_dot + 0.1*randn(1);
    x7_3 = z_3 + 0.1*randn(1);
    x8_3 = z_3_dot + 0.1*randn(1);
    x9_3 = x_3 + 0.1*randn(1);
    x10_3 = x_3_dot + 0.1*randn(1);
    x11_3 = y_3 + 0.1*randn(1);
    x12_3 = y_3_dot + 0.1*randn(1);
    
    % download the desired positions for three drones at chaque moment
    xd_1 = p_1(t,1); yd_1 = p_1(t,2); zd_1 = p_1(t,3);
    xd_2 = p_2(t,1); yd_2 = p_2(t,2); zd_2 = p_2(t,3);
    xd_3 = p_3(t,1); yd_3 = p_3(t,2); zd_3 = p_3(t,3);
    
    % Gains (same values for three drones) used in the backstepping control for the dynamique modele, see illustration in pdf attached
    [k1,k2,k3,k4,k5,k6,k7,k8,k9,k10,k11,k12] = deal(1);
    
    % backstepping control process for 3 drones(same structure, but repete 3 times):
    %drone1:
    % for Position system part
    ez_1_dot = x8_1 - zd_1_v;
    e2_1 = x8_1 - zd_1_v + k1 * (x7_1 - zd_1);
    ex_1_dot = x10_1 - xd_1_v;
    e3_1 = x10_1 - xd_1_v + k3 * (x9_1 - xd_1);
    ey_1_dot = x12_1 - yd_1_v;
    e4_1 = x12_1 - yd_1_v + k5 * (x11_1 - yd_1);
    
    u1_1 = m * (g - zd_1_a + k1 * ez_1_dot + k2 * e2_1) / (cos(x1_1) * cos(x3_1));
    ux_1 = m * (-xd_1_a + k3 * ex_1_dot + k4 * e3_1) / u1_1;
    uy_1 = m * (-yd_1_a + k5 * ey_1_dot + k6 * e4_1) / u1_1;

    % for Pose system part
    ephi_1_dot = x2_1 - dphid_1;
    e5_1 = x2_1 - dphid_1 + k7 * (x1_1 - phid_1);
    etheta_1_dot = x4_1 - dthetad_1;
    e6_1 = x4_1 - dthetad_1 + k9 * (x3_1 - thetad_1);
    epsi_1_dot = x6_1 - dpsid_1;
    e7_1 = x6_1 - dpsid_1 + k11 * (x5_1 - psid_1);
    
    u2_1 = (-x4_1 * x6_1 * a1 + x4_1 * b1 + ddphid_1 - k7 * ephi_1_dot - k8 * e5_1) / c1;
    u3_1 = (-x2_1 * x6_1 * a2 + x2_1 * b2 + ddthetad_1 - k9 * etheta_1_dot - k10 * e6_1) / c2;
    u4_1 = (-x2_1 * x4_1 * a3 + ddpsid_1 - k11 * epsi_1_dot - k12 * e7_1) / c3;

    % Update of states
    phi_1_double_dot = x4_1 * x6_1 * a1 - x4_1 * b1 + c1 * u2_1;
    theta_1_double_dot = x2_1 * x6_1 * a2 - x2_1 * b2 + c2 * u3_1;
    psi_1_double_dot = x2_1 * x4_1 * a3 + c3 * u4_1;
    z_1_double_dot = g - u1_1 * cos(x1_1) * cos(x3_1) / m;
    x_1_double_dot = -u1_1 * ux_1 / m;
    y_1_double_dot = -u1_1 * uy_1 / m;

    % Euler integration to update velocities
    phi_1_dot = phi_1_dot + phi_1_double_dot * dt;
    theta_1_dot = theta_1_dot + theta_1_double_dot * dt;
    psi_1_dot = psi_1_dot + psi_1_double_dot * dt;
    z_1_dot = z_1_dot + z_1_double_dot * dt;
    x_1_dot = x_1_dot + x_1_double_dot * dt;
    y_1_dot = y_1_dot + y_1_double_dot * dt;

    % Euler integration to update states
    phi_1 = phi_1 + phi_1_dot * dt;
    theta_1 = theta_1 + theta_1_dot * dt;
    psi_1 = psi_1 + psi_1_dot * dt;
    x_1 = x_1 + x_1_dot * dt;
    y_1 = y_1 + y_1_dot * dt;
    z_1 = z_1 + z_1_dot * dt;

    % Store state values
    if t <= num_steps
        phi_1_array(t,1) = phi_1;
        theta_1_array(t) = theta_1;
        psi_1_array(t) = psi_1;
        z_1_array(t) = z_1;
        x_1_array(t) = x_1;
        y_1_array(t) = y_1;
        phi_1_dot_array(t,1) = phi_1_dot;
        theta_1_dot_array(t) = theta_1_dot;
        psi_1_dot_array(t) = psi_1_dot;
        z_1_dot_array(t) = z_1_dot;
        x_1_dot_array(t) = x_1_dot;
        y_1_dot_array(t) = y_1_dot;
    end
    
    % drone2:
    % Position system
    ez_2_dot = x8_2 - zd_2_v;
    e2_2 = x8_2 - zd_2_v + k1 * (x7_2 - zd_2);
    ex_2_dot = x10_2 - xd_2_v;
    e3_2 = x10_2 - xd_2_v + k3 * (x9_2 - xd_2);
    ey_2_dot = x12_2 - yd_2_v;
    e4_2 = x12_2 - yd_2_v + k5 * (x11_2 - yd_2);
    
    u1_2 = m * (g - zd_2_a + k1 * ez_2_dot + k2 * e2_2) / (cos(x1_2) * cos(x3_2));
    ux_2 = m * (-xd_2_a + k3 * ex_2_dot + k4 * e3_2) / u1_2;
    uy_2 = m * (-yd_2_a + k5 * ey_2_dot + k6 * e4_2) / u1_2;

    % Pose system
    ephi_2_dot = x2_2 - dphid_2;
    e5_2 = x2_2 - dphid_2 + k7 * (x1_2 - phid_2);
    etheta_2_dot = x4_2 - dthetad_2;
    e6_2 = x4_2 - dthetad_2 + k9 * (x3_2 - thetad_2);
    epsi_2_dot = x6_2 - dpsid_2;
    e7_2 = x6_2 - dpsid_2 + k11 * (x5_2 - psid_2);
    
    u2_2 = (-x4_2 * x6_2 * a1 + x4_2 * b1 + ddphid_2 - k7 * ephi_2_dot - k8 * e5_2) / c1;
    u3_2 = (-x2_2 * x6_2 * a2 + x2_2 * b2 + ddthetad_2 - k9 * etheta_2_dot - k10 * e6_2) / c2;
    u4_2 = (-x2_2 * x4_2 * a3 + ddpsid_2 - k11 * epsi_2_dot - k12 * e7_2) / c3;

    % Update of states
    phi_2_double_dot = x4_2 * x6_2 * a1 - x4_2 * b1 + c1 * u2_2;
    theta_2_double_dot = x2_2 * x6_2 * a2 - x2_2 * b2 + c2 * u3_2;
    psi_2_double_dot = x2_2 * x4_2 * a3 + c3 * u4_2;
    z_2_double_dot = g - u1_2 * cos(x1_2) * cos(x3_2) / m;
    x_2_double_dot = -u1_2 * ux_2 / m;
    y_2_double_dot = -u1_2 * uy_2 / m;

    % Euler integration to update velocities
    phi_2_dot = phi_2_dot + phi_2_double_dot * dt;
    theta_2_dot = theta_2_dot + theta_2_double_dot * dt;
    psi_2_dot = psi_2_dot + psi_2_double_dot * dt;
    z_2_dot = z_2_dot + z_2_double_dot * dt;
    x_2_dot = x_2_dot + x_2_double_dot * dt;
    y_2_dot = y_2_dot + y_2_double_dot * dt;

    % Euler integration to update states
    phi_2 = phi_2 + phi_2_dot * dt;
    theta_2 = theta_2 + theta_2_dot * dt;
    psi_2 = psi_2 + psi_2_dot * dt;
    x_2 = x_2 + x_2_dot * dt;
    y_2 = y_2 + y_2_dot * dt;
    z_2 = z_2 + z_2_dot * dt;
    
    % Store state values
    if t <= num_steps
        phi_2_array(t,1) = phi_2;
        theta_2_array(t) = theta_2;
        psi_2_array(t) = psi_2;
        z_2_array(t) = z_2;
        x_2_array(t) = x_2;
        y_2_array(t) = y_2;
        phi_2_dot_array(t,1) = phi_2_dot;
        theta_2_dot_array(t) = theta_2_dot;
        psi_2_dot_array(t) = psi_2_dot;
        z_2_dot_array(t) = z_2_dot;
        x_2_dot_array(t) = x_2_dot;
        y_2_dot_array(t) = y_2_dot;
    end
    
    %drone3:
    % Position system
    ez_3_dot = x8_3 - zd_3_v;
    e2_3 = x8_3 - zd_3_v + k1 * (x7_3 - zd_3);
    ex_3_dot = x10_3 - xd_3_v;
    e3_3 = x10_3 - xd_3_v + k3 * (x9_3 - xd_3);
    ey_3_dot = x12_3 - yd_3_v;
    e4_3 = x12_3 - yd_3_v + k5 * (x11_3 - yd_3);
    
    u1_3 = m * (g - zd_3_a + k1 * ez_3_dot + k2 * e2_3) / (cos(x1_3) * cos(x3_3));
    ux_3 = m * (-xd_3_a + k3 * ex_3_dot + k4 * e3_3) / u1_3;
    uy_3 = m * (-yd_3_a + k5 * ey_3_dot + k6 * e4_3) / u1_3;

    % Pose system
    ephi_3_dot = x2_3 - dphid_3;
    e5_3 = x2_3 - dphid_3 + k7 * (x1_3 - phid_3);
    etheta_3_dot = x4_3 - dthetad_3;
    e6_3 = x4_3 - dthetad_3 + k9 * (x3_3 - thetad_3);
    epsi_3_dot = x6_3 - dpsid_3;
    e7_3 = x6_3 - dpsid_3 + k11 * (x5_3 - psid_3);
    
    u2_3 = (-x4_3 * x6_3 * a1 + x4_3 * b1 + ddphid_3 - k7 * ephi_3_dot - k8 * e5_3) / c1;
    u3_3 = (-x2_3 * x6_3 * a2 + x2_3 * b2 + ddthetad_3 - k9 * etheta_3_dot - k10 * e6_3) / c2;
    u4_3 = (-x2_3 * x4_3 * a3 + ddpsid_3 - k11 * epsi_3_dot - k12 * e7_3) / c3;

    % Update of states
    phi_3_double_dot = x4_3 * x6_3 * a1 - x4_3 * b1 + c1 * u2_3;
    theta_3_double_dot = x2_3 * x6_3 * a2 - x2_3 * b2 + c2 * u3_3;
    psi_3_double_dot = x2_3 * x4_3 * a3 + c3 * u4_3;
    z_3_double_dot = g - u1_3 * cos(x1_3) * cos(x3_3) / m;
    x_3_double_dot = -u1_3 * ux_3 / m;
    y_3_double_dot = -u1_3 * uy_3 / m;

    % Euler integration to update velocities
    phi_3_dot = phi_3_dot + phi_3_double_dot * dt;
    theta_3_dot = theta_3_dot + theta_3_double_dot * dt;
    psi_3_dot = psi_3_dot + psi_3_double_dot * dt;
    z_3_dot = z_3_dot + z_3_double_dot * dt;
    x_3_dot = x_3_dot + x_3_double_dot * dt;
    y_3_dot = y_3_dot + y_3_double_dot * dt;

    % Euler integration to update states
    phi_3 = phi_3 + phi_3_dot * dt;
    theta_3 = theta_3 + theta_3_dot * dt;
    psi_3 = psi_3 + psi_3_dot * dt;
    x_3 = x_3 + x_3_dot * dt;
    y_3 = y_3 + y_3_dot * dt;
    z_3 = z_3 + z_3_dot * dt;

    % Store state values
    if t <= num_steps
        phi_3_array(t,1) = phi_3;
        theta_3_array(t) = theta_3;
        psi_3_array(t) = psi_3;
        z_3_array(t) = z_3;
        x_3_array(t) = x_3;
        y_3_array(t) = y_3;
        phi_3_dot_array(t,1) = phi_3_dot;
        theta_3_dot_array(t) = theta_3_dot;
        psi_3_dot_array(t) = psi_3_dot;
        z_3_dot_array(t) = z_3_dot;
        x_3_dot_array(t) = x_3_dot;
        y_3_dot_array(t) = y_3_dot;
    end
end
%% plot figures
% Plot animation

% figure(2)
% plot3(x_cmd_array, y_cmd_array, z_cmd_array);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Center of mass Trajectory Animation');
% grid on;
% axis equal;
% drawnow;

p_cm=[p_cm; p_cm(end,:)];
figure(2)
plot3(p_cm(:,1), p_cm(:,2), p_cm(:,3), '-');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajectory animation of center of mass');
grid on;
axis equal;
drawnow;

figure(3)
plot3(p_1(:,1), p_1(:,2), p_1(:,3),'-o');
hold on
plot3(p_2(:,1), p_2(:,2), p_2(:,3),'-o');
plot3(p_3(:,1), p_3(:,2), p_3(:,3),'-o');
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajectory Animation of three drones');
grid on;
axis equal;
legend('drone_1','drone_2','drone_3');

figure(4);
subplot(3, 2, 2);
plot(0:dt:(num_steps-1)*dt, phi_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, phi_2_array, '-');
plot(0:dt:(num_steps-1)*dt, phi_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('\phi');
title('Roll Angle');
legend('phi_1','phi_2','phi_3');

subplot(3, 2, 4);
plot(0:dt:(num_steps-1)*dt, theta_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, theta_2_array, '-');
plot(0:dt:(num_steps-1)*dt, theta_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('\theta');
title('Pitch Angle');
legend('theta_1','theta_2','theta_3');

subplot(3, 2, 6);
plot(0:dt:(num_steps-1)*dt, psi_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, psi_2_array, '-');
plot(0:dt:(num_steps-1)*dt, psi_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('\psi');
title('Yaw Angle');
legend('psi_1','psi_2','psi_3');

subplot(3, 2, 1);
plot(0:dt:(num_steps-1)*dt, z_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, z_2_array, '-');
plot(0:dt:(num_steps-1)*dt, z_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('Z');
title('Z Position');
legend('z_1','z_2','z_3');

subplot(3, 2, 3);
plot(0:dt:(num_steps-1)*dt, x_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, x_2_array, '-');
plot(0:dt:(num_steps-1)*dt, x_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('X');
title('X Position');
legend('x_1','x_2','x_3');

subplot(3, 2, 5);
plot(0:dt:(num_steps-1)*dt, y_1_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, y_2_array, '-');
plot(0:dt:(num_steps-1)*dt, y_3_array, '-');
hold off
xlabel('Time (s)');
ylabel('Y');
title('Y Position');
legend('y_1','y_2','y_3');

figure(5);
subplot(3, 2, 2);
plot(0:dt:(num_steps-1)*dt, phi_1_dot_array, '-','DisplayName','phi_1_dot');
hold on
plot(0:dt:(num_steps-1)*dt, phi_2_dot_array, '-','DisplayName','phi_2_dot');
plot(0:dt:(num_steps-1)*dt, phi_3_dot_array, '-','DisplayName','phi_2_dot');
hold off
xlabel('Time (s)');
ylabel('\phi_dot');
title('Speed of Roll Angle');
legend('phi_1_dot','phi_2_dot','phi_3_dot');

subplot(3, 2, 4);
plot(0:dt:(num_steps-1)*dt, theta_1_dot_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, theta_2_dot_array, '-');
plot(0:dt:(num_steps-1)*dt, theta_3_dot_array, '-');
hold off
xlabel('Time (s)');
ylabel('\theta_dot');
title('Speed of Pitch Angle');
legend('theta_1_dot','theta_2_dot','theta_3_dot');

subplot(3, 2, 6);
plot(0:dt:(num_steps-1)*dt, psi_1_dot_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, psi_2_dot_array, '-');
plot(0:dt:(num_steps-1)*dt, psi_3_dot_array, '-');
hold off
xlabel('Time (s)');
ylabel('\psi_dot');
title('Speed of Yaw Angle');
legend('psi_1_dot','psi_2_dot','psi_3_dot');

subplot(3, 2, 1);
plot(0:dt:(num_steps-1)*dt, z_1_dot_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, z_2_dot_array, '-');
plot(0:dt:(num_steps-1)*dt, z_3_dot_array, '-');
hold off
xlabel('Time (s)');
ylabel('Z_dot');
title('Speed in axe Z');
legend('z_1_dot','z_2_dot','z_3_dot')


subplot(3, 2, 3);
plot(0:dt:(num_steps-1)*dt, x_1_dot_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, x_2_dot_array, '-');
plot(0:dt:(num_steps-1)*dt, x_3_dot_array, '-');
hold off
xlabel('Time (s)');
ylabel('X_dot');
title('Speed in axe X');
legend('x_1_dot','x_2_dot','x_3_dot')

subplot(3, 2, 5);
plot(0:dt:(num_steps-1)*dt, y_1_dot_array, '-');
hold on
plot(0:dt:(num_steps-1)*dt, y_2_dot_array, '-');
plot(0:dt:(num_steps-1)*dt, y_3_dot_array, '-');
hold off
xlabel('Time (s)');
ylabel('Y_dot');
title('Speed in axe Y ');
legend('y_1_dot','y_2_dot','y_3_dot')


% % Adjust the layout
% subtitle('Drone State Variables Over Time');
% Compare the final value with noise and the value disered.
disp('final value for 6 outputs');
disp(x_1_array(num_steps));
disp(y_1_array(num_steps));
disp(z_1_array(num_steps));
disp(phi_1_array(num_steps));
disp(theta_1_array(num_steps));
disp(psi_1_array(num_steps));

figure(6)
subplot(1,3,1)
plot(0:dt:(num_steps-1)*dt,x_1_array-x_2_array - dx12, '-');
hold on
plot(0:dt:(num_steps-1)*dt, x_2_array-x_3_array - dx23, '-');
hold off
xlabel('Time (s)');
ylabel('Error on axe X');
title('Error between three drones on axe X');
legend('Error distance robot 1 (leader) robot 2','Error distance robot 2 robot 3')

subplot(1,3,2)
plot(0:dt:(num_steps-1)*dt,y_1_array-y_2_array - dy12, '-');
hold on
plot(0:dt:(num_steps-1)*dt,y_2_array-y_3_array - dy23, '-');
hold off
xlabel('Time (s)');
ylabel('Error on axe Y');
title('Error between three drones on axe Y');
legend('Error distance robot 1 (leader) robot 2','Error distance robot 2 robot 3')

subplot(1,3,3)
plot(0:dt:(num_steps-1)*dt,z_1_array-z_2_array - dz12, '-');
hold on
plot(0:dt:(num_steps-1)*dt, z_2_array-z_3_array - dz23, '-');
hold off
xlabel('Time (s)');
ylabel('Error on axe Z');
title('Error between three drones on axe Z');
legend('Error distance robot 1 (leader) robot 2','Error distance robot 2 robot 3')

figure(7)
subplot(1,2,1)
plot(0:dt:(num_steps-1)*dt,x_cmd_array, '-','LineWidth', 4);
hold on
plot(0:dt:(num_steps-1)*dt,p_1(:,1), '-','LineWidth', 1);
plot(0:dt:(num_steps-1)*dt,p_2(:,1), '-','LineWidth', 1);
plot(0:dt:(num_steps-1)*dt,p_3(:,1), '-','LineWidth', 1);
plot(0:dt:(num_steps-1)*dt,p_cm(:,1), '-','LineWidth', 2);
hold off
xlabel('Time (s)');
ylabel('Position on axe X');
title('Evolution of position on axe X');
legend('Position X desired for the center of mass','Position X of drone 1','Position X of drone 2','Position X of drone 3', 'Average Position X of 3 drones')

subplot(1,2,2)
plot(0:dt:(num_steps-1)*dt,p_cm(:,3)-z_cmd_array, '-','LineWidth', 2);
hold on
plot(0:dt:(num_steps-1)*dt,p_cm(:,2)-y_cmd_array, '-','LineWidth', 2);
plot(0:dt:(num_steps-1)*dt,p_cm(:,1)-x_cmd_array, '-','LineWidth', 3);
hold off
xlabel('Time (s)');
ylabel('Error');
title("Evolution of error detween actual center of mass and designed center of mass")
legend('gravity center error z','gravity center error y','gravity center error x')