classdef PlanarRobot8 < handle
    properties
        % Link lengths for the planar robot fingers
        finger1_link1 = 0.05;      % Length of the first link of finger 1 hi im here
        finger1_link2 = 0.05;      % Length of the second link of finger 1
        finger1_angles = [pi pi/2];   % Initial joint angles for finger 1
        
        finger2_link1 = 0.05;      % Length of the first link of finger 2
        finger2_link2 = 0.05;      % Length of the second link of finger 2
        finger2_angles = [0 pi/2];    % Initial joint angles for finger 2
        
        % Table properties
        table_x_axis_offset = -0.45; % Offset along the x-axis of the table
        table_height = 1.2988;       % Height of the table
        
        % Robot properties
        finger_1;         % Planar 2-link robot (finger 1)
        finger_2;         % Planar 2-link robot (finger 2)
        r;               % Linear UR3e robot
        
        traj_steps = 100;     % Number of steps in the trajectory
        gripper_steps = 50;   % Number of steps for gripper animation
        
        % Safety variable
        safety = false;   % Toggle for enabling safety features
        
        % Enable/Disable Gripper
        enable_gripper = false; % Toggle for enabling the gripper
        
        % Waypoints for robot movement
        waypoint_A = transl(0.3,0,1.6) * trotx(pi); %used for close side to get in wall placement pos
        % waypoint_B = transl(0,0.45,1.7) * trotx(pi) * troty(pi/2); % Target waypoint
        waypoint_C = transl(0.3,-0.3,1.6) * trotx(pi); % used for far side to get in wall placement pos
        
        %Text handle for transforms
        text_h = [];
        
        %Base transform
        base_transform = transl(0, 0, 1.2988)
    end
    
    methods
        function self = PlanarRobot8()
            % Constructor to initialize the robot model
            
            % Create robot gripper fingers if enabled
            if self.enable_gripper
                % Define finger 1 with two revolute joints
                self.finger_1 = SerialLink([ ...
                    Revolute('d', 0, 'a', self.finger1_link1, 'alpha', pi/2, 'standard'), ...
                    Revolute('d', 0, 'a', self.finger1_link2, 'alpha', 0, 'standard')], ...
                    'name', 'finger 1');
                self.finger_1.delay = 0;
                
                
                % Define finger 2 with two revolute joints
                self.finger_2 = SerialLink([ ...
                    Revolute('d', 0, 'a', self.finger2_link1, 'alpha', pi/2, 'standard'), ...
                    Revolute('d', 0, 'a', self.finger2_link2, 'alpha', 0, 'standard')], ...
                    'name', 'finger 2');
                self.finger_2.delay = 0;
            end
            
            % Create a new figure window that is docked
            figure('WindowStyle', 'docked');
            hold on;
            
            % Create and place the concrete floor
            surf([-4, -4; 4, 4], ...
                [-4, 4; -4, 4], ...
                [0.01, 0.01; 0.01, 0.01], ...
                'CData', imread('concrete.jpg'), ...
                'FaceColor', 'texturemap');
            
            % Place the table in the scene
            table_h = PlaceObject("mytable1.ply", [self.table_x_axis_offset 0 0]);
            verts = [get(table_h, 'Vertices'), ones(size(get(table_h, 'Vertices'), 1), 1)] * trotz(pi/2);
            set(table_h, 'Vertices', verts(:, 1:3));
            
            % Conditionally place safety features if safety is enabled
            if self.safety
                % Place barriers
                barrier1_h = PlaceObject("mybarrier1.ply", [-2, 0, 0.35]); %#ok<NASGU>
                barrier2_h = PlaceObject("mybarrier1.ply", [2, 0, 0.35]); %#ok<NASGU>
                barrier3_h = PlaceObject('mybarrier1.ply', [2, 0, 0.35]);
                verts = [get(barrier3_h, 'Vertices'), ones(size(get(barrier3_h, 'Vertices'), 1), 1)] * trotz(0.5 * pi);
                set(barrier3_h, 'Vertices', verts(:, 1:3));
                barrier4_h = PlaceObject('mybarrier1.ply', [-2, 0, 0.35]);
                verts = [get(barrier4_h, 'Vertices'), ones(size(get(barrier4_h, 'Vertices'), 1), 1)] * trotz(pi/2);
                set(barrier4_h, 'Vertices', verts(:, 1:3));
                
                % Place emergency stop button
                emergencystop_h = PlaceObject("emergencyStopButton.ply", [-3 0 0]); %#ok<NASGU>
                
                % Place man
                man_h = PlaceObject("personMaleConstruction.ply", [-3.5 0 0.01]); %#ok<NASGU>
                
                % Place fire extinguisher
                fireextiguisher_h = PlaceObject("fireExtinguisher.ply", [-3 1 0]); %#ok<NASGU>
                
            end
            
            
            % Initialize the UR3e robot
            self.r = LinearUR3e;
            self.r.model.base = self.base_transform * trotx(pi/2) * troty(pi/2);
            self.r.model.animate([0 0 0 0 0 0 0]);
            
            axis([-4 2 -2 2 0 2])
            
        end
        
        function openGripper(self)
            % Define the opening angles for the gripper fingers
            % Adjust these angles as needed to simulate the opening
            open_angles_finger1 = [pi pi/2]; % Example angles
            open_angles_finger2 = [0 pi/2]; % Example angles
            
            
            % Create a trajectory for the fingers to move to the open angles
            finger1Matrix = jtraj(self.finger1_angles, open_angles_finger1, self.gripper_steps);
            finger2Matrix = jtraj(self.finger2_angles, open_angles_finger2, self.gripper_steps);
            
            % Animate the fingers to open
            for i = 1:self.gripper_steps
                % Update and plot the planar robots
                self.finger_1.plot(finger1Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                self.finger_2.plot(finger2Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                
                % Update the plot
                drawnow;
            end
            % Update the class properties with the new angles
            self.finger1_angles = open_angles_finger1;
            self.finger2_angles = open_angles_finger2;
        end
        
        function closeGripper(self)
            % Define the closing angles for the gripper fingers
            % Adjust these angles as needed to simulate the closing
            close_angles_finger1 = [pi 3*pi/4];  % Example angles for finger 1 fully closed
            close_angles_finger2 = [0 3*pi/4];  % Example angles for finger 2 fully closed
            
            % Create a trajectory for the fingers to move to the closed angles
            finger1Matrix = jtraj(self.finger1_angles, close_angles_finger1, self.gripper_steps);
            finger2Matrix = jtraj(self.finger2_angles, close_angles_finger2, self.gripper_steps);
            
            % Animate the fingers to close
            for i = 1:self.gripper_steps
                % Update and plot the planar robots
                self.finger_1.plot(finger1Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                self.finger_2.plot(finger2Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                
                % Update the plot
                drawnow;
            end
            
            % Update the angles stored in the object after closing
            self.finger1_angles = close_angles_finger1;
            self.finger2_angles = close_angles_finger2;
        end
        
        
        function animateRobot(self)
            T1 = transl(0.3, 0.16, 1.4);
            T2 = transl(0.3, 0.16, 1.5);
            T3 = transl(0.2, 0.25, 1.6);  % Example additional transformation

            % Automatically collect all variables starting with 'T' that are transformations
            T_variables = who('T*');

            % Initialize an empty array to store the transforms
            T_Array = [];

            % Loop through each transformation and concatenate it along the 3rd dimension
            for i = 1:length(T_variables)
                T_Array = cat(3, T_Array, eval(T_variables{i}));
            end
            
            % T1 = transl(0.3,0.16,1.4)
            % T2 = transl(0.3,0.16,1.5)



            % %% ADD UR TRANSFORMS IN HERE
      
            
            % T_Array = cat(3, T1, T2)
            
            
            
            for i = 1:size(T_Array, 3)
                
                if ~isempty(self.text_h)
                    delete(self.text_h);
                end
                
                message = num2str(self.r.model.fkine(self.r.model.getpos).T, '%.2f');
                self.text_h = text(1, 1, 1,  message, 'FontSize', 15, 'Color', [0 0 0]);
                    
                % Display the current step in the transformation sequence
                fprintf('On transform step %d of %d\n', i, size(T_Array, 3));
                
                
                % Compute the inverse kinematics for the current transformation
                q = self.r.model.ikcon(T_Array(:,:,i));
                qMatrix = jtraj(self.r.model.getpos, q, self.traj_steps);
                
                % Animate the robot through the trajectory
                for j = 1:self.traj_steps
                    % Animate the UR3e robot at each step
                    self.r.model.animate(qMatrix(j, :));
                    
                    if self.enable_gripper
                        % Update the planar robot base using forward kinematics
                        self.finger_1.base = self.r.model.fkineUTS(qMatrix(j,:));
                        self.finger_2.base = self.r.model.fkineUTS(qMatrix(j,:));
                        
                        % Plot the planar robot fingers with current angles
                        self.finger_1.plot(self.finger1_angles, 'nowrist', 'noname', 'noshadow', 'nobase');
                        self.finger_2.plot(self.finger2_angles, 'nowrist', 'noname', 'noshadow', 'nobase');
                    end
                    
                    
                    
                    % Set fixed axis limits for the plot
                    axis([-2 2 -2 2 0 2]);
                    drawnow;
                end
                
                % Control gripper opening and closing based on step index
                if self.enable_gripper
                    if mod(i, 4) == 0
                        % Open the gripper every 4 steps
                        disp("Opening gripper");
                        self.openGripper();
                    end
                    
                    if mod(i - 2, 4) == 0
                        % Close the gripper every 4 steps, offset by 2 steps
                        disp("Closing gripper");
                        self.closeGripper();
                    end
                end
                
                % Check and display the discrepancy between planned and actual end-effector positions
                currentTransformEndEffector = self.r.model.fkine(self.r.model.getpos).T;
                realEndEffectorCoords = currentTransformEndEffector(1:3,4);
                plannedEndEffectorCoords = T_Array(1:3,4,i);
                fprintf('IK Discrepancy (m) = %.6f\n\n', norm(realEndEffectorCoords - plannedEndEffectorCoords));
            end

        end
    end
end