%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = true;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

positions = [0;0];


% num_drones = 1;
num_drones = 2;

count = 1;
%instantiate a drone object, input the axis and arena limits
drones = [];
drones = [drones Drone(ax1, spaceDim, num_drones, 0)];
drones = [drones Drone(ax1, spaceDim, num_drones, 0.3)];


ButtonMove = uicontrol('Parent',f1,'Style','pushbutton', ...
'String','Move!', 'ForegroundColor','#F6F6F6','BackgroundColor', ...
'#2ECC71','FontWeight', 'bold', 'Units','normalized','Position',[0.1 0.1 0.10 0.08], ...
'Visible', 'on', 'Callback', {@sendMoveCommand, drones(1)});

while(drones(1).time < 1000.0)
    %clear axis
    cla(ax1);
    %update and draw drones
    for i = 1:num_drones
        if ~drones(i).ready
            set(ButtonMove,'Enable','off');
        else
            set(ButtonMove,'Enable','on');
        end
        if (i == 1)
            update(drones(i), 0, drones(i+1));
        end
    end
    if (num_drones == 2)
        update(drones(2), 1, drones(1));
    end
%     positions = [positions, [(drones(i).pos(1)); drones(i).pos(2)]];
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

function sendMoveCommand(src, event, drone)
    drone.respondToMoveCommand();
end
