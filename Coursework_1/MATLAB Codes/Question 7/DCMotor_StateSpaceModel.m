% Question 7
% DCMotor_StateSpaceModel.m
%
%
function [x_DC_new, theta_dot] = DCMotor_StateSpaceModel(SamplingTime, V, x_DC)
    % Input:  V
    % Output: theta_dot (Motor angular velocity)
    % State:  x_DC = [i; theta_dot]

    % Constants:
    R   = 2.0;              % Resistance            (Ohms)
    L   = 0.5;              % Inductance            (Henrys)
    K_m = 0.1;              % Torque constant
    K_e = 0.1;              % Back emf constant
    b   = 0.2;              % Friction coefficient  (Nms)
    J   = 0.02;             % Moment of Inertia     (kg.m^2/s^2)
    
    A = [-R/L, -K_e/L; K_m/J, -b/J];
    B = [1/L; 0];
    
    % x[t + 1]  = exp(A*t)*x[t] + Inverse(A)*(exp(A*t) - I)*B*u[t]
    x_DC_new    = expm(A.*SamplingTime)*x_DC + A\(expm(A.*SamplingTime) - eye(size(A)))*B*V;
    
    % y[t + 1]  = [0, 1]*x[t]
    theta_dot   = [0, 1]*x_DC;
    
end