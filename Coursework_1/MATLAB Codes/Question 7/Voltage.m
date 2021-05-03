% Question 7
% Voltage.m
%
%
function [V_L, V_R] = Voltage(t, V_L, mode)
    if mode == 1
        V_R = V_L;
        
    elseif mode == 2
        V_R = 2*V_L;
        
    elseif mode == 3
        V_R = V_L + V_L*sin(t);
        
    end
end