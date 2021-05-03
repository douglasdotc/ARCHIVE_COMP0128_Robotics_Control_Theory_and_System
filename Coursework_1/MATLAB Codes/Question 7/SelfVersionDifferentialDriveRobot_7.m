% Question 7
% SelfVersionDifferentialDriveRobot.m
%
%
% Initialize Workspace:
clc;
clear all;
close all;

% Init parameters:
init_7

for t = 0:SamplingTime:TotalTime
    % Voltage:
    % mode = 1 --> V_R = V_L
    % mode = 2 --> V_R = 2*V_L
    % mode = 3 --> V_R = V_L + V_L*sin(t)
    mode = 3;
    [V_L, V_R] = Voltage(t, V_L, mode);
    
    % DC Motor Model:
    % Left:
    [x_DC(1:2), theta_dot_L] = DCMotor_StateSpaceModel(SamplingTime, V_L, x_DC(1:2));
    s_L = theta_dot_L*r; % Left wheel velocity
    
    % Right:
    [x_DC(3:4), theta_dot_R] = DCMotor_StateSpaceModel(SamplingTime, V_R, x_DC(3:4));
    s_R = theta_dot_R*r; % Right wheel velocity
    
    % Robot Orientation:
    phi_dot = (s_R - s_L)/l_W;
    
    % Robot Centre:
    s_M     = 0.5*(s_L + s_R); % Robot centre velocity
    
    % Forward Kinematics:
    p_x_dot = s_M*cos(phi);
    p_y_dot = s_M*sin(phi);
    
    if phi_dot == 0
        % Handle singular case (phi_dot == 0 --> robot moving in straight line,
        % phi is constant, p_x, p_y is linear:
        p_x = p_x + p_x_dot*SamplingTime;
        p_y = p_y + p_y_dot*SamplingTime;
        phi = phi;

    else
        % phi_dot != 0 --> robot turning:
        p_x = p_x + (s_M/phi_dot)*[sin(phi_dot*SamplingTime + phi) - sin(phi)];
        p_y = p_y - (s_M/phi_dot)*[cos(phi_dot*SamplingTime + phi) - cos(phi)];
        phi = phi + phi_dot*SamplingTime;

    end
    
    % Append new position to memory arrays so the trajectory can be plotted
    % later:
    p_x_mem(end + 1) = p_x;
    p_y_mem(end + 1) = p_y;
    
end

% Plot trajectory:
plot(p_x_mem, p_y_mem)
xlabel('x')
ylabel('y')