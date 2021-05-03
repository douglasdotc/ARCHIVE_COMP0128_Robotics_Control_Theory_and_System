% Question 2
% controller_question_2.m
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