classdef FrankaER < RobotBaseClass
    properties(Access =public)   
        plyFileNameStem = 'FrankaER1';
    end
    methods (Access = public) 
        function self = FrankaER(baseTr)
			self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
            axis([-0.25, 0.25, -0.25, 0.25, 0, 1.5]);
        end
        function CreateModel(self)       
            link(1) = Link('d', 0.333, 'a', 0.0000, 'alpha', -pi/2,'offset',0);
            link(2) = Link('d', 0.000, 'a', 0.0000, 'alpha', pi/2, 'offset',0);
            link(3) = Link('d', 0.316, 'a', 0.0000, 'alpha', pi/2, 'offset',0);
            link(4) = Link('d', 0.000, 'a', -0.0825, 'alpha', -pi/2, 'offset',0);
            link(5) = Link('d', 0.384, 'a', 0.0825,'alpha', pi/2, 'offset',0);
            link(6) = Link('d', 0.0000, 'a', 0.0000, 'alpha', pi/2, 'offset',0);
            link(7) = Link('d', 0.107, 'a', 0.0880, 'alpha', 0.00, 'offset',0);
            link(2).qlim = [deg2rad(-30) deg2rad(30)];
            self.model = SerialLink(link,'name',self.name);
        end   
    end
end