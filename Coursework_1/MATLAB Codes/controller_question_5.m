% Question 5 with 6
% controller_question_5w6.m
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
p_x_mem(end + 1) = p_x;
p_y_mem(end + 1) = p_y;

% !! Feedback line:
% Combine r and p --> Controller:
r_rp    = atan2(r_y(k) - p_y, r_x(k) - p_x);

% Combine r_rp and phi to get the error angle epsilon between the robot 
% orientation and desination:
epsilon = r_rp - phi;

% PID:
phi_dot_correction = K_p*epsilon;

u = [4 - phi_dot_correction; 4 + phi_dot_correction];

% !! Update to r(k + 1) if reached r(k):
if norm([r_x(k); r_y(k)] - [p_x; p_y]) <= 0.01 && k < size(r_x, 2)
    k = k + 1;
end

% !! Terminate when the last coordinate is reached:
if norm([r_x(k); r_y(k)] - [p_x; p_y]) <= 0.01 && k == size(r_x, 2)
    u = [0; 0];
    if plotted == false
        % Question 6: Plot:
        axes2       = axes('Parent',figure);
        %   1. Desired robot trajectory
        r_traj      = plot(axes2,[0, r_x], [0, r_y]);
        hold on
        %   2. Robot trajectory estimated from odometry measurements
        odom_traj   = plot(axes2, p_x_mem, p_y_mem);
        
        %   3. True robot trajectory
        true_traj   = plot(axes2, csim.Log.Output(2,:), csim.Log.Output(3,:));
        legend('Desired robot trajectory','Robot trajectory estimated from odometry measurements','True robot trajectory', 'Location', 'southoutside')
        xlabel('x')
        ylabel('y')
        title('Question 6 plots')
        hold off
        
        plotted = true;
    end
end

% !! States update;
y_prev = y;