classdef LinearDobot5 < RobotBaseClass
    %% DobotMagician Placed on linear rail

    properties(Access = public)              
        plyFileNameStem = 'LinearDobot5';
    end
    
    methods
%% Define robot Function 
        function self = LinearDobot5(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            link(3) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            link(4) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            link(5) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-180),deg2rad(180)]);
            link(6) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',pi, 'qlim',[deg2rad(-85),deg2rad(85)]);

            
            % Incorporate joint limits
            link(1).qlim = [-2.3 -0.01]; %Linear Rail (Joint 0)
            link(2).qlim = [deg2rad(-135), deg2rad(135)];  % Base rotation (Joint 1)
            link(3).qlim = [deg2rad(5), deg2rad(80)];      % Shoulder rotation (Joint 2)
            link(4).qlim = [deg2rad(-5), deg2rad(85)];     % Elbow rotation (Joint 3)
            link(5).qlim = [deg2rad(-180), deg2rad(180)];  % Wrist rotation (Joint 4)
            link(6).qlim = [deg2rad(-85), deg2rad(85)];    % Tool rotation or adjustment (Joint 5)

            
        
            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end
