% Question 7
% init_7.m
%
%
r            = 0.0671;      % Wheel radius
l_W          = 0.235;       % Distance between wheels
x_DC         = [0;0;0;0];   % State
phi          = 0;           % Robot orientation
p_x          = 0;           % x-position
p_y          = 0;           % y-position
p_x_mem      = [];          % Array storing the x-position
p_y_mem      = [];          % Array storing the y-position
V_L          = 4;           % DC Motor Voltage
TotalTime    = 20;          % Total timew
SamplingTime = 0.02;        % Sampling time