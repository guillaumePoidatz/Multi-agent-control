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
xd = 10;  xd_v = 0;  xd_a = 0;
yd = 10;  yd_v = 0;  yd_a = 0;
zd = 10;  zd_v = 0;  zd_a = 0;

phid = 2;  dphid = 0;  ddphid=0;
thetad = 2;  dthetad = 0;  ddthetad=0;
psid = 0;  dpsid = 0;  ddpsid=0;

% Gains
[k1,k2,k3,k4,k5,k6,k7,k8,k9,k10,k11,k12] = deal(1);


%% Trajectory
% New trajectory parameters
a_traj_x = 5;  % The long half-axis of the elliptical trajectory
b_traj_y = 3;  % The short half-axis of an elliptical trajectory
theta_traj = pi/4;  % Angle of rotation of elliptical trajectory

% Add panning offset
x_offset = 10;
y_offset = 5;


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
    
    % Calculate the elliptical trajectory position and add a translation offset
    xd = a_traj_x * cos(t*dt) * cos(theta_traj) - b_traj_y * sin(t*dt) * sin(theta_traj) + x_offset;
    yd = a_traj_x * cos(t*dt) * sin(theta_traj) + b_traj_y * sin(t*dt) * cos(theta_traj) + y_offset;
    zd = 2;  % A fixed height is still used here
    
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
    
    ephi_dot = x2 - dphid;
    e5 = x2 - dphid + k7 * (x1 - phid);
    etheta_dot = x4 - dthetad;
    e6 = x4 - dthetad + k9 * (x3 - thetad);
    epsi_dot = x6 - dpsid;
    e7 = x6 - dpsid + k11 * (x5 - psid);
    
    u2 = (-x4 * x6 * a1 + x4 * b1 + ddphid - k7 * ephi_dot - k8 * e5) / c1;
    u3 = (-x2 * x6 * a2 + x2 * b2 + ddthetad - k9 * etheta_dot - k10 * e6) / c2;
    u4 = (-x2 * x4 * a3 + ddpsid - k11 * epsi_dot - k12 * e7) / c3;

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
plot(0:dt:(num_steps-1)*dt, phi_array, '-');
xlabel('Time (s)');
ylabel('\phi');
title('Roll Angle');

subplot(3, 2, 4);
plot(0:dt:(num_steps-1)*dt, theta_array, '-');
xlabel('Time (s)');
ylabel('\theta');
title('Pitch Angle');

subplot(3, 2, 6);
plot(0:dt:(num_steps-1)*dt, psi_array, '-');
xlabel('Time (s)');
ylabel('\psi');
title('Yaw Angle');

subplot(3, 2, 1);
plot(0:dt:(num_steps-1)*dt, z_array, '-');
xlabel('Time (s)');
ylabel('Z');
title('Z Position');

subplot(3, 2, 3);
plot(0:dt:(num_steps-1)*dt, x_array, '-');
xlabel('Time (s)');
ylabel('X');
title('X Position');

subplot(3, 2, 5);
plot(0:dt:(num_steps-1)*dt, y_array, '-');
xlabel('Time (s)');
ylabel('Y');
title('Y Position');

% Adjust the layout
sgtitle('Drone State Variables Over Time');
% Compare the final value with noise and the value disered.
disp('final value for 6 outputs');
disp(x_array(num_steps));
disp(y_array(num_steps));
disp(z_array(num_steps));
disp(phi_array(num_steps));
disp(theta_array(num_steps));
disp(psi_array(num_steps));
