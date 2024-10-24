classdef PlanarRobot9 < handle
    properties
        % Link lengths for the planar robot fingers
        finger1_link1 = 0.05;
        finger1_link2 = 0.05;
        finger1_angles = [pi pi/2];

        finger2_link1 = 0.05;
        finger2_link2 = 0.05;
        finger2_angles = [0 pi/2];

        % Table properties
        table_x_axis_offset = -0.45; 
        table_height = 1.2988;       

        % Robot properties
        finger_1;        
        finger_2;        
        r;               

        % Ball positions and handles
        golfball_h;     
        golfball_pos;   
        
        traj_steps = 100;     
        gripper_steps = 50;   

        % Safety variable
        safety = false;   

        % Enable/Disable Gripper
        enable_gripper = false; 

        % Waypoints for robot movement
        waypoint_A = transl(0.3,0,1.6) * trotx(pi); 
        waypoint_C = transl(0.3,-0.3,1.6) * trotx(pi); 

        % Text handle for transforms
        text_h = []; 

        % Base transform
        base_transform = transl(1.3, 0.6, 1.2988);
    end

    methods
        function self = PlanarRobot9()
            % Constructor to initialize the robot model
            self.createFingers();
            self.createEnvironment();
            self.initializeGolfBall();
            self.initializeUR3e();
        end
        
        function createFingers(self)
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
        end
        
        function createEnvironment(self)
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
                % Place barriers and other safety features
                self.placeSafetyFeatures();
            end
            
            axis([-4 2 -2 2 0 2]);  % Set axis limits
        end
        
        function placeSafetyFeatures(self)
            % Implementation for placing safety features
            barrier1_h = PlaceObject("mybarrier1.ply", [-2, 0, 0.35]); 
            barrier2_h = PlaceObject("mybarrier1.ply", [2, 0, 0.35]); 
            barrier3_h = PlaceObject('mybarrier1.ply', [2, 0, 0.35]);
            barrier4_h = PlaceObject('mybarrier1.ply', [-2, 0, 0.35]);
            % Add other features similarly...
        end
        
        function initializeGolfBall(self)
            % Golf ball prop
            self.golfball_pos = [1.2 0.2 (self.table_height + 0.05)];
            self.golfball_h = PlaceObject('golfball.ply', self.golfball_pos);
        end
        
        function initializeUR3e(self)
            % Initialize the UR3e robot
            self.r = LinearDobot5;  % Make sure to replace with your robot class
            self.r.model.base = self.base_transform * trotx(pi/2) * troty(pi/2);
            self.r.model.animate([0 0 0 0 0 0]);
        end
        
        function openGripper(self)
            % Define the opening angles for the gripper fingers
            open_angles_finger1 = [pi pi/2];
            open_angles_finger2 = [0 pi/2];

            % Create a trajectory for the fingers to move to the open angles
            finger1Matrix = jtraj(self.finger1_angles, open_angles_finger1, self.gripper_steps);
            finger2Matrix = jtraj(self.finger2_angles, open_angles_finger2, self.gripper_steps);
            
            % Animate the fingers to open
            for i = 1:self.gripper_steps
                self.finger_1.plot(finger1Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                self.finger_2.plot(finger2Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                drawnow;
            end
            
            self.finger1_angles = open_angles_finger1;
            self.finger2_angles = open_angles_finger2;
        end
        
        function closeGripper(self)
            % Define the closing angles for the gripper fingers
            close_angles_finger1 = [pi 3*pi/4];
            close_angles_finger2 = [0 3*pi/4];
            
            % Create a trajectory for the fingers to move to the closed angles
            finger1Matrix = jtraj(self.finger1_angles, close_angles_finger1, self.gripper_steps);
            finger2Matrix = jtraj(self.finger2_angles, close_angles_finger2, self.gripper_steps);
            
            % Animate the fingers to close
            for i = 1:self.gripper_steps
                self.finger_1.plot(finger1Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                self.finger_2.plot(finger2Matrix(i, :), 'nowrist', 'noname', 'noshadow', 'nobase');
                drawnow;
            end
            
            self.finger1_angles = close_angles_finger1;
            self.finger2_angles = close_angles_finger2;
        end
        
        function animateRobot(self)
            % Similar implementation as your original animateRobot method...
        end
    end
end
