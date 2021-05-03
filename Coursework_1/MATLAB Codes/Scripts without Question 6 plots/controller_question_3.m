% Question 3
% controller_question_3.m
%
%
% !! Distance travelled by wheels, using odometry:
d_l = (2*pi*R/64)*(y(1) - y_prev(1));
d_r = (2*pi*R/64)*(y(2) - y_prev(2));

% !! Forward Kinematics:
alpha   = (d_r - d_l)/l_w;      % Robot angle relative to origin
phi_dot = alpha/dt;             % Rate of change of robot orientation
s_m     = 0.5*(d_l + d_r)/dt;   % Robot centre velocity

% ** Find p_x, p_y, phi:
if phi_dot == 0
    % Handle sigular case (phi_dot == 0 --> robot moving in straitght line,
    % phi is constant, p_x, p_y is linear:
    p_x = p_x + s_m*cos(phi)*dt;
    p_y = p_y + s_m*sin(phi)*dt;
    phi = phi;
    
else
    % phi_dot != 0 --> robot turning:
    p_x = p_x + (s_m/phi_dot)*[sin(phi_dot*dt + phi) - sin(phi)];
    p_y = p_y - (s_m/phi_dot)*[cos(phi_dot*dt + phi) - cos(phi)];
    phi = phi + phi_dot*dt;
    
end

% !! Feedback line:
% Combine r and p --> Controller:
r_rp = atan2(r(2) - p_y, r(1) - p_x);

% Combine r_rp and phi to get the error angle epsilon between the robot 
% orientation and desination:
epsilon = r_rp - phi;

% PID:
phi_dot_correction = K_p*epsilon;

u = [4 - phi_dot_correction; 4 + phi_dot_correction];

% !! Terminate if reached:
if norm(r - [p_x; p_y]) <= 0.01
    u = [0; 0];
end

% !! States update;
y_prev = y;