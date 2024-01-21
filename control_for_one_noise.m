clear all;
close all;
clc;

%% Parameters of drone
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

%% Initialization
% Initial position, velocity, acceleration, and pose
x = 0;  x_dot = 0;
y = 0;  y_dot = 0;
z = 0;  z_dot = 0;

phi = 0;  phi_dot = 0;
theta = 0;  theta_dot = 0;
psi = 0;  psi_dot = 0;

% Target position, velocity, acceleration, and pose
xd = 2;  xd_v = 0;  xd_a = 0;
yd = 2;  yd_v = 0;  yd_a = 0;
zd = 2;  zd_v = 0;  zd_a = 0;

phid = 2;  phid_dot = 0;  phid_double_dot = 0;
thetad = 2;  thetad_dot = 0;  thetad_double_dot = 0;
psid = 0;  psid_dot = 0;  psid_double_dot = 0;

% Gains
k1 = 1;
k2 = 1;
k3 = 1;
k4 = 1;
k5 = 1;
k6 = 1;
k7 = 1;
k8 = 1;
k9 = 1;
k10 = 1;
k11 = 1;
k12 = 1;

%% Definition of State Space
% States
x1 = phi;
x2 = phi_dot;
x3 = theta;
x4 = theta_dot;
x5 = psi;
x6 = psi_dot;
x7 = z;
x8 = z_dot;
x9 = x;
x10 = x_dot;  
x11 = y;
x12 = y_dot;

%% Loop

num_steps = 200;
dt = 0.1;  % Adjusted time step for better animation

% Arrays to store state values
phi_array = zeros(num_steps, 1);
theta_array = zeros(num_steps, 1);
psi_array = zeros(num_steps, 1);
z_array = zeros(num_steps, 1);
x_array = zeros(num_steps, 1);
y_array = zeros(num_steps, 1);

figure;

for t = 1:num_steps
%     % Without noises
%     x1 = phi;
%     x2 = phi_dot;
%     x3 = theta;
%     x4 = theta_dot;
%     x5 = psi;
%     x6 = psi_dot;
%     x7 = z;
%     x8 = z_dot;
%     x9 = x;
%     x10 = x_dot;
%     x11 = y;
%     x12 = y_dot;
    
    % Adding noises to states
    x1 = phi+0.1*randn(1);
    x2 = phi_dot+0.1*randn(1);
    x3 = theta+0.1*randn(1);
    x4 = theta_dot+0.1*randn(1);
    x5 = psi+0.1*randn(1);
    x6 = psi_dot+0.1*randn(1);
    x7 = z+0*randn(1);
    x8 = z_dot+0.1*randn(1);
    x9 = x+0.1*randn(1);
    x10 = x_dot+0.1*randn(1);
    x11 = y+0.1*randn(1);
    x12 = y_dot+0.1*randn(1);
    
    % Position system
    ez_dot = x8 - zd_v;
    e2 = x8 - zd_v + k1 * (x7 - zd);
    ex_dot = x10 - xd_v;
    e3 = x10 - xd_v + k3 * (x9 - xd);
    ey_dot = x12 - yd_v;
    e4 = x12 - yd_v + k5 * (x11 - yd);
    
    u1 = m * (g - zd_a + k1 * ez_dot + k2 * e2) / (cos(x1) * cos(x3));
    ux = m * (-xd_a + k3 * ex_dot + k4 * e3) / u1;
    uy = m * (-yd_a + k5 * ey_dot + k6 * e4) / u1;

    % Pose system
    phid = asin(ux * sin(x5) - uy * cos(x5));
    thetad = asin(ux / (cos(x1) * cos(x5)) - sin(x1) * sin(x5) / (cos(x1) * cos(x5)));
    
    ephi_dot = x2 - phid_dot;
    e5 = x2 - phid_dot + k7 * (x1 - phid);
    etheta_dot = x4 - thetad_dot;
    e6 = x4 - thetad_dot + k9 * (x3 - thetad);
    epsi_dot = x6 - psid_dot;
    e7 = x6 - psid_dot + k11 * (x5 - psid);
    
    u2 = (-x4 * x6 * a1 + x4 * b1 + phid_double_dot - k7 * ephi_dot - k8 * e5) / c1;
    u3 = (-x2 * x6 * a2 + x2 * b2 + thetad_double_dot - k9 * etheta_dot - k10 * e6) / c2;
    u4 = (-x2 * x4 * a3 + psid_double_dot - k11 * epsi_dot - k12 * e7) / c3;

    % Update of states
    phi_double_dot = x4 * x6 * a1 - x4 * b1 + c1 * u2;
    theta_double_dot = x2 * x6 * a2 - x2 * b2 + c2 * u3;
    psi_double_dot = x2 * x4 * a3 + c3 * u4;
    z_double_dot = g - u1 * cos(x1) * cos(x3) / m;
    x_double_dot = -u1 * ux / m;
    y_double_dot = -u1 * uy / m;

    % Euler integration to update velocities
    phi_dot = phi_dot + phi_double_dot * dt;
    theta_dot = theta_dot + theta_double_dot * dt;
    psi_dot = psi_dot + psi_double_dot * dt;
    z_dot = z_dot + z_double_dot * dt;
    x_dot = x_dot + x_double_dot * dt;
    y_dot = y_dot + y_double_dot * dt;

    % Euler integration to update states
    phi = phi + phi_dot * dt;
    theta = theta + theta_dot * dt;
    psi = psi + psi_dot * dt;
    z = z + z_dot * dt;
    x = x + x_dot * dt;
    y = y + y_dot * dt;
    
    % Store state values
    if t <= num_steps
        phi_array(t,1) = phi;
        theta_array(t) = theta;
        psi_array(t) = psi;
        z_array(t) = z;
        x_array(t) = x;
        y_array(t) = y;
    end

    % Plot animation
    plot3(x_array, y_array, z_array, '-o');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Drone Trajectory Animation');
    grid on;
    axis equal;
    drawnow;
end

%% Create subplots for each state variable
figure;

subplot(3, 2, 2);
plot(0:dt:(num_steps-1)*dt, phi_array, '-o');
xlabel('Time (s)');
ylabel('\phi');
title('Roll Angle');

subplot(3, 2, 4);
plot(0:dt:(num_steps-1)*dt, theta_array, '-o');
xlabel('Time (s)');
ylabel('\theta');
title('Pitch Angle');

subplot(3, 2, 6);
plot(0:dt:(num_steps-1)*dt, psi_array, '-o');
xlabel('Time (s)');
ylabel('\psi');
title('Yaw Angle');

subplot(3, 2, 1);
plot(0:dt:(num_steps-1)*dt, z_array, '-o');
xlabel('Time (s)');
ylabel('Z');
title('Altitude');

subplot(3, 2, 3);
plot(0:dt:(num_steps-1)*dt, x_array, '-o');
xlabel('Time (s)');
ylabel('X');
title('X Position');

subplot(3, 2, 5);
plot(0:dt:(num_steps-1)*dt, y_array, '-o');
xlabel('Time (s)');
ylabel('Y');
title('Y Position');

% Adjust the layout
sgtitle('Drone State Variables Over Time');