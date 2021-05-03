%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0]; %0.6 before
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
        
        %Physical parameters used in the model
        m = 1.305; % mass
        kd = 0.01; % drag constant
        g = [0; 0; -9.8]; 
        I = [0.0221 0 0; 0 0.0259 0; 0 0 0.0354]; %Inertia matrix with values for I_xx, I_yy and I_zz respectively
        
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        distances
        
        arm_len
        
        first_ang
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos;
        
        state;
        
        %drone angles
        theta; 
        
        %drone velocities
        xdot;
        
        count2;
        
        %drone angular velocities
        omega;
        
        %drone rotation matrix
        R
        
        %Controller parameters
        integral_err_z_pos
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones
        
        %ready state of the drone
        ready
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones, follower)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [follower;0;0];
                
                obj.theta = zeros(3, 1); 
                
                obj.arm_len = sqrt(0.6^2 + 0.6^2);
                
                obj.xdot = zeros(3, 1);
                
                obj.count2 = 1;
                
                obj.omega = zeros(3, 1);
                
                obj.first_ang = 2*pi / 100;
                
                obj.state = 0;
                
                obj.distances = [0];
                
                obj.integral_err_z_pos = 0;
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                obj.ready = true;
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            if(obj.drone_follow)
               axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = first_move(obj)                                          
            [T_B, torques] = altitudeController(obj, 2.5);
                        
            evolveStates(obj, T_B, torques);
            
            disp("Position: ");
            disp(obj.pos);
            
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
            
            if abs(obj.pos(3) - 2.5) > 0.05
                obj.ready = false;
            elseif abs(obj.pos(3) - 2.5) < 0.05
                obj.ready = true;
            end
        end
        
        function obj = second_move_circle(obj)         
            [T_B, torques] = circleController(obj);
            
            evolveStates(obj, T_B, torques);
            
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
                    
            obj.ready = false;           
        end   
        
        function obj = second_move_down(obj)
            [T_B, torques] = altitudeController(obj, 0);
            
            %When the drone is very closed to the ground, turn off the
            %thrust
            
            evolveStates(obj, T_B, torques);
            
            if obj.pos(3) < 0.005
                T_B(3) = 0;
                obj.pos(3) = 0;
                obj.xdot(1) = 0;
                obj.xdot(2) = 0;
                obj.xdot(3) = 0;
            end 
            
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
            
            obj.ready = false;
        end
      
        function obj = evolveStates(obj, T_B, torques)
            % Rotation matrix using Z-Y-X Euler Representation
            R_z = [cos(obj.theta(1)), -sin(obj.theta(1)), 0; sin(obj.theta(1)), cos(obj.theta(1)), 0; 0, 0, 1];
            R_y = [cos(obj.theta(2)), 0, sin(obj.theta(2)); 0, 1, 0; -sin(obj.theta(2)), 0, cos(obj.theta(2))];
            R_x = [1, 0, 0; 0, cos(obj.theta(3)), -sin(obj.theta(3)); 0, sin(obj.theta(3)), cos(obj.theta(3))];
            R_mat = R_z * R_y * R_x;
            
            a = obj.g + 1 / obj.m * (R_mat*(T_B)) - obj.kd .* obj.xdot;
            if obj.pos(3) <= 0 && T_B(3) == 0
                a = 0;
            end
            omegadot = inv(obj.I) * (torques - cross(obj.omega, obj.I * obj.omega));  
            
            obj.omega = obj.omega + obj.time_interval * omegadot;
            thetadot = inv([1 0 -sin(obj.theta(2)); 0 cos(obj.theta(1)) cos(obj.theta(2))*sin(obj.theta(1)); 0 -sin(obj.theta(1)) cos(obj.theta(2))*cos(obj.theta(1))]) * obj.omega;
            obj.theta = obj.theta + obj.time_interval * thetadot;
            obj.xdot = obj.xdot + obj.time_interval * a;
            obj.pos = obj.pos + obj.time_interval * obj.xdot;
            
            if obj.pos(3) <= 0 %So that, the drone could not penetrate into the ground, and stop when it hits the floor
                obj.pos(3) = 0;
                obj.xdot(3) = 0;
            end
        end
        
        function [T_B, torques] = altitudeController(obj, desired_z_pos)
            % PID gains parameters for the thrust
            Kp1 = 15.0;%20.0;
            Kd1 = 6.0;
            Ki1 = 6.0; %6.0    
            
            % PID gains parameters for the torques
            Kp2 = 5;
            Kd2 = 5;
            
            %Controller parameters for landing
            if desired_z_pos == 0
                Kp1 = 1;
                Kd1 = 10;
                Ki1 = 1;
                Kp2 = 1;
            	Kd2 = 1;
                desired_z_pos = obj.pos(3) - 0.0001;
            end
            
            % equilibrium points
            desired_z_dot = 0;
            desired_theta = [0; 0; 0];
            desired_omega = [0; 0; 0];
            
            % control the thrust in body frame, namely T_B_z and T_B
            integral_z_pos = obj.integral_err_z_pos + (obj.pos(3) - desired_z_pos) * obj.time_interval;
            T_B_z = -Kp1*(obj.pos(3) - desired_z_pos) - Kd1*(obj.xdot(3) - desired_z_dot) - Ki1*integral_z_pos;
            obj.integral_err_z_pos = integral_z_pos;
            T_B = [0; 0; T_B_z];
            
            % control the torque
            torques = -Kp2*(obj.theta - desired_theta) - Kd2*(obj.omega - desired_omega); 
        end
        
        function [T_B, torques] = circleController(obj)
            % Thrust controller
            % Control the Thrust and Torque as Inputs
            Kp1 = 15.0;%20.0;
            Kd1 = 6.0;
            Ki1 = 6.0; %6.0
            
            % Torque controller 
            Kp2 = 10; % was 10
            Kd2 = 2;
            
            % equilibrium points
            desired_z_pos = 2.5;
            desired_z_dot = 0;
                       
            point = [(cos(obj.first_ang) * 2.5) - 2.5; (sin(obj.first_ang) * 2.5)];
            scatter(point(1), point(2));
            diff_x = (point(1)+20) - (obj.pos(1)+20);
            diff_y = (point(2)+20) - (obj.pos(2)+20);            
            overall_distance = sqrt(diff_x^2 + diff_y^2);
           
            % T_B and T_B_z are thrust in body frame, change here
            integral_z_pos = obj.integral_err_z_pos + (obj.pos(3) - desired_z_pos) * obj.time_interval;
            T_B_z = -Kp1*(obj.pos(3) - desired_z_pos) - Kd1*(obj.xdot(3) - desired_z_dot) - Ki1*integral_z_pos;
            
            obj.integral_err_z_pos = integral_z_pos;
            T_B = [0; 0; T_B_z];
                   
            mid_val = diff_x;
            end_val = -diff_y;
            t1 = 0.8 * mid_val;
            t2 = 0.8 * end_val;
            desired_theta = [0; t1 - 0.5 * obj.xdot(1); t2 + 0.5 * obj.xdot(2)];
            desired_omega = [0; 0; 0];

            % change torque
            torques = -Kp2*(obj.theta - desired_theta) - Kd2*(obj.omega - desired_omega);
            
            if (overall_distance < 0.25)
                if (obj.count2 < 100)
                    obj.first_ang = obj.first_ang + 2*pi / 100;
                    display(obj.count2)
                    obj.count2 = obj.count2 + 1; 
                else
                    obj.state = 3;
                end
            end
        end
        
        function obj = fllo_move(obj, other_drone)     
            I_xx = 0.0221; % arbitrary values for inertia matrix 
            I_yy = 0.0259;
            I_zz = 0.0354;
           
            m = 1.305; % mass
            kd = 0.0613; % drag constant
            g = [0; 0; -9.8]; 
            R_z = [cos(obj.theta(1)), -sin(obj.theta(1)), 0; sin(obj.theta(1)), cos(obj.theta(1)), 0; 0, 0, 1];
            R_y = [cos(obj.theta(2)), 0, sin(obj.theta(2)); 0, 1, 0; -sin(obj.theta(2)), 0, cos(obj.theta(2))];
            R_x = [1, 0, 0; 0, cos(obj.theta(3)), -sin(obj.theta(3)); 0, sin(obj.theta(3)), cos(obj.theta(3))];
            % intertia matrix
            R_mat = R_z * R_y * R_x;
            I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];
            % Thrust controller
            % Control the Thrust and Torque as Inputs
            Kp1 = 15.0;%20.0;
            Kd1 = 6.0;
            Ki1 = 6.0; %6.0
            % Torque controller 
            Kp2 = 10; % was 10
            Kd2 = 2;
            % equilibrium points
            desired_z_pos = 2.5;
            desired_z_dot = 0;
            
            if (obj.time > 10 && sqrt(other_drone.pos(1)^2 + other_drone.pos(2)^2) < 0.3)
                point = [0.3; 0];
            else
                point = [other_drone.pos(1); other_drone.pos(2)];
            end
            
            %0.21213203435

            diff_x = (point(1)+20) - (obj.pos(1)+20);
            diff_y = (point(2)+20) - (obj.pos(2)+20);
            
            drone_dist = sqrt((other_drone.pos(1) - obj.pos(1))^2 + (other_drone.pos(2) - obj.pos(2))^2);
            display(drone_dist)
            
            obj.distances = [obj.distances, drone_dist];
            

            % T_B and T_B_z are thrust in body frame, change here
            integral_z_pos = obj.integral_err_z_pos + (obj.pos(3) - desired_z_pos) * obj.time_interval;
            T_B_z = -Kp1*(obj.pos(3) - desired_z_pos) - Kd1*(obj.xdot(3) - desired_z_dot) - Ki1*integral_z_pos;
            
            obj.integral_err_z_pos = integral_z_pos;
            T_B = [0; 0; T_B_z];
            
            a = g + (1 / m) * (R_mat*(T_B)) - kd .* obj.xdot;

            mid_val = diff_x;
            end_val = -diff_y;
            
            t1 = (0.8 * mid_val);
            t2 = (0.8 * end_val);
           
            desired_theta = [0; t1 - 0.5 * obj.xdot(1); t2 + 0.5 * obj.xdot(2)];

            desired_omega = [0; 0; 0];

            % change torque
            torques = -Kp2*(obj.theta - desired_theta) - Kd2*(obj.omega - desired_omega); 
            % Angular acceleration
            omegadot = inv(I) * (torques - cross(obj.omega, I * obj.omega));       
            
            % Change all physical parameters accordingly
            obj.omega = obj.omega + obj.time_interval * omegadot;
            thetadot = inv([1 0 -sin(obj.theta(2)); 0 cos(obj.theta(1)) cos(obj.theta(2))*sin(obj.theta(1)); 0 -sin(obj.theta(1)) cos(obj.theta(2))*cos(obj.theta(1))]) * obj.omega;
            obj.theta = obj.theta + obj.time_interval * thetadot;
            obj.xdot = obj.xdot + obj.time_interval * a;
            obj.pos = obj.pos + obj.time_interval * obj.xdot;
            
            rot_mat = eul2rotm([obj.theta(1), obj.theta(2), obj.theta(3)]);
            obj.R = rot_mat;
        end   
        
        function update(obj, fllo, other_drone)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
            if (fllo == 0)
                if (obj.state == 1)
                    obj = first_move(obj);
                elseif (obj.state == 2)
                    obj = second_move_circle(obj);
                elseif (obj.state == 3)
                    obj = second_move_down(obj);
                end
            elseif (fllo == 1)
                if (other_drone.state == 1)
                    obj = first_move(obj);
                elseif (other_drone.state == 2)
                    obj = fllo_move(obj, other_drone);
                elseif (other_drone.state == 3)
                    obj = second_move_down(obj);
                end
            end
            %draw drone on figure
            draw(obj);
        end
        
        function respondToMoveCommand(obj)
            if obj.state == 0
                obj.state = 1;
            elseif obj.state == 1
                obj.state = 2;
            end
        end
    end
end
