% Question 4 with 6
% init_question_4w6.m
%
%
% !! Constants !!
% ** Wheel Radius (Using straight line simulation)
%   R = x_last_dist / last_odom_value * 64 * 1/2pi
%   x_last_dist = 2.2207, last_odom_value = 337
R               = 0.0671;

% ** Distance between wheels (Using stop one wheel simulation)
%   2pi*l_w = last_odom_value * 2piR/64
%   last_odom_value = 228, R = 0.0671
l_w             = 0.235;

% !! Initialize !!
r               = [1; 1];       % Desired Position
plot(r(1), r(2), '*')
u               = [4; 4];       % Voltage
y_prev          = [0; 0];       % robot odometry memory

p_x             = 0;            % x-position
p_y             = 0;            % y-position
phi             = 0;            % robot orientation
Pos_reached     = false;        % Flag for position reached
plotted         = false;        % Flag for plotting graphs: Q6
p_x_mem         = [];           % Memory for p_x
p_y_mem         = [];           % Memory for p_y
r_mem           = [0; 0];       % Memory for r with init [0; 0]
% PID Gains:
K_p             = 8;            % position