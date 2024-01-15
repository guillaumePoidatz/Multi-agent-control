% This function is used to calculate the positions of three drones
% The inputs are the total period, step length, the inition positions of the drone and desired position of the center of mass
% The output is the respective position of the drone

function [p1, p2, p3,Pcm_array] = multi_agent_positions_calculs_v2(x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_cm_array, y_cm_array, z_cm_array, dx12, dy12, dz12, dx23, dy23, dz23, nb_steps, dt)
%% Initialisation
num_steps = nb_steps;

% allocate space for the vectors which represent the initial position of the drones
[p1, p2, p3] = deal(zeros(num_steps, 3));
Pcm_array = [];

% la position de point de masse centrale
Pcmd_array = [x_cm_array, y_cm_array, z_cm_array];

% distances designed between each drone 
dp12 = [dx12, dy12, dz12];
dp23 = [dx23, dy23, dz23];

% the initial positions of drones
p1(1,1) = x_1;
p1(1,2) = y_1;
p1(1,3) = z_1;

p2(1,1) = x_2;
p2(1,2) = y_2;
p2(1,3) = z_2;

p3(1,1) = x_3;
p3(1,2) = y_3;
p3(1,3) = z_3;



%% main loop
for k = 1:num_steps - 1
    
    % at each moment k, calculating the actual position of center of mass
    Xcm = 1/3*(p1(k,1) + p2(k,1) + p3(k,1));
    Ycm = 1/3*(p1(k,2) + p2(k,2) + p3(k,2));
    Zcm = 1/3*(p1(k,3) + p2(k,3) + p3(k,3));
    Pcm = [Xcm, Ycm, Zcm];
    Pcm_array = [Pcm_array; Pcm];
    
    gain = 50;
    
    % u1, u2, u3 are speeds of drones, defined by user as the iuputs, we have u1 = x1_dot, u2 = x2_dot, u3 = x3_dot.
    % we take the drone_1 as the leader, so u is a term of u1
    
    %faux
%     u1 = 50*(p2(k,:)-p1(k,:) + dp12) -3 *gain*min((Pcm - Pcmd_array(k,:)),10);
%     u2 = 50*(p1(k,:) - p2(k,:) - dp12) + 50*(p3(k,:) - p2(k,:) - dp23);
%     u3 = 50*(p2(k,:) - p3(k,:) + dp23);
    
    u1 = 50*(p2(k,:)-p1(k,:) + dp12) -3 *gain*min((Pcm - Pcmd_array(k,:)),10);
    u2 = 50*(p1(k,:) - p2(k,:) - dp12) + 50*(p3(k,:) - p2(k,:) + dp23);
    u3 = 50*(p2(k,:) - p3(k,:) - dp23);

%     u1 = 2*(p2(k,:)-p1(k,:) + dp12) -0.1 *gain*min((Pcm - Pcmd_array(k,:)),10);
%     u2 = 2*(p1(k,:) - p2(k,:) - dp12) + 2*(p3(k,:) - p2(k,:) + dp23);
%     u3 = 2*(p2(k,:) - p3(k,:) - dp23);


    
    % update the position by using the equation: x(k+1) = x(k) + v*delta_t  
    p1(k+1,:) = p1(k,:) + dt * u1;
    p2(k+1,:) = p2(k,:) + dt * u2;
    p3(k+1,:) = p3(k,:) + dt * u3;
    
end
%% figures
% figure(2)
% plot3(p1(:,1), p1(:,2), p1(:,3),'-o');
% hold on
% plot3(p2(:,1), p2(:,2), p2(:,3),'-o');
% plot3(p3(:,1), p3(:,2), p3(:,3),'-o');
% hold off
end