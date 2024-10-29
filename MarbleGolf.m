classdef MarbleGolf< handle
    properties
        % Link lengths for the planar robot fingers
        finger1_link1 = 0.05;      % Length of the first link of finger 1
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
        f; %FrankaER
 
        traj_steps = 100;     % Number of steps in the trajectory
        gripper_steps = 50;   % Number of steps for gripper animation
 
        % Safety variable
        safety = false;   % Toggle for enabling safety features
 
        % Enable/Disable Gripperß
        enable_gripper = false; % Toggle for enabling the gripper
 
        % golfball positions and handles
        golfball_1_h; % Handle for golfball 1
        golfball_1_pos; % Coordinates of golfball 1
        
        golfball_positions; % Array to store all golfball positions
        golfball_handles; % Array to store all golfball handles
        original_vertices; % Store the original vertices of the golfball
 
 
        % golfball dimension properties
        golfball_length = 0.13; % Length of each golfball
        golfball_height = 0.034; % Height of each golfball
 
        % Waypoints for robot movement
        waypoint_A = transl(-0.2,0.4,2) * trotx(pi); %used for close side to get in wall placement pos
        waypoint_B = transl(-0.3,0.2,1.5) * trotx(pi) * troty(pi/2); % Target waypoint
        waypoint_C = transl(-0.6,0.4,2) * trotx(pi); % used for far side to get in wall placement pos
 
        %Text handle for transforms
        text_h = [];
 
        %Base transform Dobot
        dobot_base_transform = transl(-0.285, 0.34, 1.2988)

        %Base transform Franka
        franka_base_transform = transl(0, -0.15, 1.2988)

        % Arduino Variables
        %a = arduino("COM3", "UNO");

        %hole variables
        holes = [-0.5,-0.25,1.3788;
        -0.4,-0.12,1.3788;
        -0.33,-0.24,1.3788;
        -0.30,-0.05,1.3788;
        -0.26, -0.17,1.3788
        ]
        hole_radius = 0.1

    end
    
    methods
        function self = MarbleGolf()
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

            % Place alley
            alley_h = PlaceObject("alley.ply", [0 self.table_height + 0.02 1.2]);
            verts = [get(alley_h, 'Vertices'), ones(size(get(alley_h, 'Vertices'), 1), 1)] * trotx(-pi/2) * trotz(pi/2);
            set(alley_h, 'Vertices', verts(:, 1:3));
 
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
            
            % Define positions for golfball
            self.golfball_1_pos = [-1.2 -0.2 (self.table_height+0.06+0.02)];

            
 
            % Concatenate golfball positions to form a 3D array
            self.golfball_positions = cat(3, ...
            self.golfball_1_pos);
 
            % Place golfballs in the scene
            self.golfball_1_h = PlaceObject('golfball2.ply', self.golfball_1_pos);
            
 
            % Store golfball handles in an array
            self.golfball_handles = [
            self.golfball_1_h
            ];
 
            % Store the original vertices of a golfball
            self.original_vertices = get(self.golfball_1_h, 'Vertices');
 
            % Initialize the UR3e robot
            self.r = LinearDobot5;
            self.r.model.base = self.dobot_base_transform * trotx(pi/2) * troty(pi/2);
            self.r.model.animate([0 0 0 0 0 0]);
 
            % Initialize the Franka robot
            self.f = FrankaER;
            self.f.model.base = self.franka_base_transform;
            self.f.model.animate([0 0 0 0 0 0 0]);
 
            % PlaceObject("golfball.ply", [0 0 0])
 
            axis([-2 2 -2 2 0 3])
 
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
        
        function generatePointCloud(self)
        
            stepRads = deg2rad(60); % Step size for incrementing the joint angles
            qlim = self.r.model.qlim; % Get joint limits for all 7 joints
            pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1)); % Total number of possible combinations
            pointCloud = zeros(pointCloudeSize, 3); % Preallocate point cloud matrix
            counter = 1;
            tic
            
            % Loop through all 7 joints
            for q1 = qlim(1,1):0.1:qlim(1,2)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    for q7 = qlim(7,1):stepRads:qlim(7,2)
                                        q = [q1, q2, q3, q4, q5, q6, q7]; % Joint configuration
                                        tr = self.r.model.fkineUTS(q); % Forward kinematics
                                        pointCloud(counter,:) = tr(1:3, 4)'; % Store (x, y, z) coordinates
                                        counter = counter + 1; % Increment counter
 
                                        % Display progress
                                        if mod(counter/pointCloudeSize * 100, 1) == 0
                                            disp(['After ', num2str(toc), ' seconds, completed ', num2str(counter/pointCloudeSize * 100 / 8 ), '% of poses']);
                                        end
                                    end
                                end
                            end
                        end
                    end
                    plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
        
                end
            end
        end

        function playGame(self)
            figure(2);

            self.animateMarble()
            self.animateFranka()
            self.animateDobot()
        end

        function animateMarble(self)
            figure(2);
            % Marble position
            % golfball_1_pos;
            % Define marble's path (for now assume it moves in a straight line)
            self.golfball_1_pos = [-1.2 ,-0.05 + (-0.33 + 0.05) * rand ,(self.table_height+0.06+0.02)];
            marble_path = linspace(self.golfball_1_pos(1), self.golfball_1_pos(1) + 1.04, 50);
            hole_radius = 0.01;


    
            % Loop over the path
            for i = 1:length(marble_path);
                % Update marble position
                delete(self.golfball_1_h);
                self.golfball_1_pos(1) = marble_path(i); % Assume it moves along x-axis
                % disp(self.golfball_1_pos);
                self.golfball_1_h = PlaceObject("golfball2.ply", self.golfball_1_pos);

                % Pause to allow visualization of movement
                pause(0.01); % Adjust pause duration for desired speed


                
                % Check if marble is near any hole
                for j = 1:size(self.holes, 1)
                    distance_to_hole = norm(self.golfball_1_pos - self.holes(j, :));
                    if distance_to_hole <= 0.02
                        % fprintf('Marble fell into hole %d at position (%.2f, %.2f)\n', j, self.holes(j, :));
                        disp('Marble fell into hole! You Scored!');
                         
                        % Teleport marble to bottom of the hole (e.g., z = -1)
                        % teleport_marble_to_hole(self.holes(j, :));
                        return; % End simulation once marble falls into a hole
                    end
                end



                
                % Visualize or simulate the marble moving
                % (Add plotting code if desired)
            end
                %desired_pos = transl(self.golfball_1_pos(1), self.golfball_1_pos(2), self.golfball_1_pos(3)) * trotx(pi);
                %q = self.f.model.ikcon(desired_pos)
            
                %qMatrix = jtraj(self.f.model.getpos, q, self.traj_steps);
            
                %for j = 1:self.traj_steps
                    % Animate the UR3e robot at each step
                    %self.f.model.animate(qMatrix(j, :));
                    %drawnow;
                %end
        end
 
 
 
       function animateDobot(self)
     
        % % Concatenate golfball positions to form a 3D array
        % self.golfball_positions = cat(3, ...
        %     self.golfball_1_pos);
    
        %     % Place golfballs in the scene
        %     % self.golfball_1_h = PlaceObject('golfball2.ply', self.golfball_1_pos);
            
    
        %     % Store golfball handles in an array
        %     self.golfball_handles = [
        %     self.golfball_1_h
        %     ];
             
        %     % Store the original vertices of a golfball
        %     self.original_vertices = get(self.golfball_1_h, 'Vertices');

            % Define transformations for waypoints and objects
            % Make sure to go to waypoint_# every second step
            T1 = self.waypoint_A;

            % T2 = transl(self.golfball_1_pos) * transl(0, 0, self.finger1_link1) * trotx(pi);
            T2 = transl(-0.2, 0.2, 1.7) * trotx(pi);
            % Transformation for waypoint C
            T3 = self.waypoint_C;
        
            % Transformation for wall 1 position
            T4 = transl(-1.2, 0.07, 1.3) * trotx(pi);
        
            % Transformation for waypoint A waiting
            % T5 = self.waypoint_A;
            T5 = transl(-1.2, 0.3,self.table_height);
        
            % Concatenate all transformations into a 3D array
            T_Array = cat(3, T1, T2, T3, T4, T5);
 
 
 
            for i = 1:size(T_Array, 3)
 
                
 
                message = num2str(self.r.model.fkine(self.r.model.getpos).T, '%.2f');
                %self.text_h = text(1, 1, 1,  message, 'FontSize', 15, 'Color', [0 0 0]);
 
                
 
                % Display the current step in the transformation sequence
                fprintf('On transform step %d of %d\n', i, size(T_Array, 3));
 
                %Logging robot task (moving to waypoint)
                if mod(i, 2) ~= 0
                    disp("Step Objective: Moving to Waypoint")
                end
 
                %Logging robot task (placing golfball in wall)
                if mod(i, 4) == 0
                    disp("Step Objective: Deploying golfball in wall strucuture")
                end
 
                %Logging robot task (Locating golfball)
                if mod(i-2, 4) == 0
                    disp("Step Objective: Locating golfball")
                end
 
 
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
            
                    % Update the golfball's position every 4 steps
                    if mod(i, 4) == 3 || mod(i, 4) == 0
                        % Determine the correct golfball handle based on the index
                        golfball_index = ceil(i / 4);
            
                        % Get the end-effector pose of the UR3e
                        end_effector_pose = self.r.model.fkine(self.r.model.getpos).T;
            
                        % Reset the golfball's vertices to their original positions
                        set(self.golfball_handles(golfball_index), 'Vertices', self.original_vertices);
            
                        % Get the current vertices of the golfball
                        vertices = get(self.golfball_handles(golfball_index), 'Vertices');
                        golfballVerticesHom = [vertices, ones(size(vertices,1),1)];
            
                        % Apply the transformation to each vertex
                        transformed_vertices = [end_effector_pose * transl(-self.golfball_1_pos(1), -self.golfball_1_pos(2), -self.table_height + self.finger1_link1 - 0.1) * golfballVerticesHom']'; %#ok<NBRAK1>
            
                        % Update the golfball's position with the transformed vertices
                        set(self.golfball_handles(golfball_index), 'Vertices', transformed_vertices(:, 1:3));
                    end
            
                    % Set fixed axis limits for the plot
                    %axis([-2 2 -2 2 0 2]);
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
                        disp("golfball located: Closing gripper");
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

       function animateFranka(self)
                % Concatenate golfball positions to form a 3D array
                self.golfball_positions = cat(3, ...
                    self.golfball_1_pos);
            
                    % Place golfballs in the scene
                    % self.golfball_1_h = PlaceObject('golfball2.ply', self.golfball_1_pos);
                    
            
                    % Store golfball handles in an array
                    self.golfball_handles = [
                    self.golfball_1_h
                    ];
                     
                    % Store the original vertices of a golfball
                    self.original_vertices = get(self.golfball_1_h, 'Vertices');

            % Define transformations for waypoints and objects
            % Make sure to go to waypoint_# every second step

            T1 = self.waypoint_A;

            T2 = transl(self.golfball_1_pos) * transl(0, 0, self.finger1_link1) * trotx(pi);
        
            % Transformation for waypoint C
            T3 = self.waypoint_C;
        
            % Transformation for wall 1 position
            T4 = transl(-0.2, 0.2, 1.7) * trotx(pi);
        
            % Transformation for waypoint A waiting
            % T5 = self.waypoint_C;
        
            % Concatenate all transformations into a 3D array
            T_Array = cat(3, T1, T2, T3, T4);



            for i = 1:size(T_Array, 3)

                

                message = num2str(self.f.model.fkine(self.f.model.getpos).T, '%.2f');
                %self.text_h = text(1, 1, 1,  message, 'FontSize', 15, 'Color', [0 0 0]);

                

                % Display the current step in the transformation sequence
                fprintf('On transform step %d of %d\n', i, size(T_Array, 3));

                %Logging robot task (moving to waypoint)
                if mod(i, 2) ~= 0
                    disp("Step Objective: Moving to Waypoint")
                end

                %Logging robot task (placing golfball in wall)
                if mod(i, 4) == 0
                    disp("Step Objective: Deploying golfball in wall strucuture")
                end

                %Logging robot task (Locating golfball)
                if mod(i-2, 4) == 0
                    disp("Step Objective: Locating golfball")
                end


                % Compute the inverse kinematics for the current transformation
                q = self.f.model.ikcon(T_Array(:,:,i));
                qMatrix = jtraj(self.f.model.getpos, q, self.traj_steps);
            
                % Animate the robot through the trajectory
                for j = 1:self.traj_steps
                    % Animate the UR3e robot at each step
                    self.f.model.animate(qMatrix(j, :));
            
                    if self.enable_gripper
                        % Update the planar robot base using forward kinematics
                        self.finger_1.base = self.f.model.fkineUTS(qMatrix(j,:));
                        self.finger_2.base = self.f.model.fkineUTS(qMatrix(j,:));
            
                        % Plot the planar robot fingers with current angles
                        self.finger_1.plot(self.finger1_angles, 'nowrist', 'noname', 'noshadow', 'nobase');
                        self.finger_2.plot(self.finger2_angles, 'nowrist', 'noname', 'noshadow', 'nobase');
                    end
            
                    % Update the golfball's position every 4 steps
                    if mod(i, 4) == 3 || mod(i, 4) == 0
                        % Determine the correct golfball handle based on the index
                        golfball_index = ceil(i / 4);
            
                        % Get the end-effector pose of the UR3e
                        end_effector_pose = self.f.model.fkine(self.f.model.getpos).T;
            
                        % Reset the golfball's vertices to their original positions
                        set(self.golfball_handles(golfball_index), 'Vertices', self.original_vertices);
            
                        % Get the current vertices of the golfball
                        vertices = get(self.golfball_handles(golfball_index), 'Vertices');
                        golfballVerticesHom = [vertices, ones(size(vertices,1),1)];
            
                        % Apply the transformation to each vertex
                        transformed_vertices = [end_effector_pose * transl(-self.golfball_1_pos(1), -self.golfball_1_pos(2), -self.table_height + self.finger1_link1 - 0.1) * golfballVerticesHom']'; %#ok<NBRAK1>
            
                        % Update the golfball's position with the transformed vertices
                        set(self.golfball_handles(golfball_index), 'Vertices', transformed_vertices(:, 1:3));
                    end
            
                    % Set fixed axis limits for the plot
                    %axis([-2 2 -2 2 0 2]);
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
                        disp("golfball located: Closing gripper");
                        self.closeGripper();
                    end
                end
            
                % Check and display the discrepancy between planned and actual end-effector positions
                currentTransformEndEffector = self.f.model.fkine(self.f.model.getpos).T;
                realEndEffectorCoords = currentTransformEndEffector(1:3,4);
                plannedEndEffectorCoords = T_Array(1:3,4,i);
                fprintf('IK Discrepancy (m) = %.6f\n\n', norm(realEndEffectorCoords - plannedEndEffectorCoords));
            end
        end

       function obj = YourClassName()
            % Constructor where you initialize the properties
            obj.a = arduino("COM3", "UNO");
            
            obj.buttonTimer = timer;
            obj.buttonTimer.ExecutionMode = "fixedRate";
            obj.buttonTimer.Period = 0.1;
            obj.buttonTimer.TimerFcn = @(~,~) eStopCheck([], [], obj.a);
            
            start(obj.buttonTimer);
        end

       function eStopCheck(~, ~, a)
            buttonState = readDigitalPin(a, "D7")
            if buttonState == 1
                disp('eStop Pressed');
            end
        end

        function obj = timer()
            % Constructor where you initialize the properties
            obj.a = arduino("COM3", "UNO");
            
            obj.buttonTimer = timer;
            obj.buttonTimer.ExecutionMode = "fixedRate";
            obj.buttonTimer.Period = 0.1;
            obj.buttonTimer.TimerFcn = @(~,~) eStopCheck([], [], obj.a);
            
            start(obj.buttonTimer);
        end

            % In MarbleGolf.m
        function newPosition = moveEndEffector(obj, deltaX, deltaY, deltaZ)
            % Apply translational offset using transl
            translationMatrix = transl(deltaX, deltaY, deltaZ);
            obj.currentTransform = obj.currentTransform * translationMatrix;
            
            % Extract the new position
            newPosition = transl(obj.currentTransform);
    
            % Update the robot model plot to reflect the new position
            if ishandle(obj.plotHandle)
                obj.plot(obj.currentTransform); % Re-plot with the new transformation
            else
                % If no existing plot, create one
                obj.plotHandle = plot(obj.robot, obj.currentTransform);
            end
        end
   end
end