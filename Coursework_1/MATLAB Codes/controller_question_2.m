% Question 2 with 6
% controller_question_2w6.m
%
%
% !! Random movements:
% Random movement errors:
e_L = y(2) - y(1);
e_R = y(1) - y(2);

% Random movement corrections:
u_L_Corr = K * e_L;
u_R_Corr = K * e_R;
u_Corr   = [u_L_Corr; u_R_Corr];

% Random movement feedback:
u = [4; 4] + u_Corr;

% Calculate r and p for plotting:
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
p_x_mem(end + 1) = p_x;
p_y_mem(end + 1) = p_y;

if plotted == false && i == csim.TotalSteps
    % Question 6: Plot:
    axes2       = axes('Parent', figure);
    %   1. Desired robot trajectory
    r_traj      = plot(axes2, r(1,:), r(2,:));
    hold on
    %   2. Robot trajectory estimated from odometry measurements
    odom_traj   = plot(axes2, p_x_mem, p_y_mem);
    
    %   3. True robot trajectory
    true_traj   = plot(axes2, csim.Log.Output(2,:), csim.Log.Output(3,:));
    legend('Desired robot trajectory','Robot trajectory estimated from odometry measurements','True robot trajectory', 'Location', 'southoutside')
    xlabel('x')
    ylabel('y')
    xlim([0 5])
    ylim([-2 2])
    title('Question 6 plots')
    
    plotted = true;
end

y_prev = y;