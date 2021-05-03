% Differential drive robot
clc;
clear all;
close all;

% Simulation parameters
TOTAL_TIME  = 60;
dt          = 0.02;
TIME_SCALE  = 0.01; % slows down simulation if > 1, speeds up if < 1 (and if computation allows...)

% Initialise Simulation
robot = DifferentialDriveDCMotor;
robot.setState(zeros(9,1));
robot.setInput([0;0]);
robot.updateOutput;

csim = ControlSimulator(robot,TOTAL_TIME,dt);

figure;
ax1 = axes;
hold(ax1,'on');
axis('equal')
axis([-0.4 4.6 -2 2])
axis('manual')
xlabel('x');
ylabel('y');
ax1.Toolbar.Visible = 'off';
ax1.Interactions = [];
robot.initPlot(ax1);
trajplot = plot(csim.Log.Output(2,:),csim.Log.Output(3,:),'linewidth',1);
robot.plot;


%%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
init_question_x;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = nan(1,csim.TotalSteps-1);
% Run Simulation
for i = 2:csim.TotalSteps
    tic
    
    % Read odometry sensors
    y = robot.Output(4:5);
    
    % Control: determine dc motor inputs. assume controller only has access
    % to the odometry measurements variable 'y'
    
    %%%%%%%%%%%%%%%%%%%%%%%% IMPLEMENT THIS SCRIPT %%%%%%%%%%%%%%%%%%%%%%%%
    controller_question_x;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulation
    csim.step(u);
    set(trajplot,'XData',csim.Log.Output(2,:));
    set(trajplot,'YData',csim.Log.Output(3,:));
    robot.plot;
    drawnow nocallbacks limitrate
    time(i-1) = toc;
    pause(TIME_SCALE*dt-toc);
end