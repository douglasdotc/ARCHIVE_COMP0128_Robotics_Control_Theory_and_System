% Question 2 with 6
% init_question_2w6.m
%
%
% !! Constants !!
% ** Wheel Radius (Using straight line simulation)
%   R = x_last_dist / last_odom_value * 64 * 1/2pi
%   x_last_dist = 2.2207, last_odom_value = 337
R       = 0.0671;

% ** Distance between wheels (Using stop one wheel simulation)
%   2pi*l_w = last_odom_value * 2piR/64
%   last_odom_value = 228, R = 0.0671
l_w     = 0.235;

% !! Initialize !!
u       = [4;4];                    % Voltage
r       = [0:1:6; zeros(1,7)];      % Desired path
y_prev  = [0; 0];                   % robot odometry memory
p_x     = 0;                        % x-position
p_y     = 0;                        % y-position
phi     = 0;                        % robot orientation
plotted = false;                    % Flag for plotting graphs: Q6
p_x_mem = [];                       % Memory for p_x
p_y_mem = [];                       % Memory for p_y

% Constants
K = 8;