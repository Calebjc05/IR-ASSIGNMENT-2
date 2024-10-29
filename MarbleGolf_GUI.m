classdef MarbleGolf_GUI < handle
    properties
        hFig              % Handle for the main figure
        hPositionText     % Handle for position display text
        hPauseButton      % Handle for the pause/play button
        robotInstance     % Stores the robot instance
        currentPosition   % Current position of the end effector
        x = 0;
    end
 
    methods
        function obj = MarbleGolf_GUI
            % Create a dummy figure and immediately close it to clear the figure state
            figure;
            close;
        
            % Initialize the current position of the end effector
            currentPosition = [0, 0, 0]; % Example initial position
            setappdata(0, 'isPaused', false);
            
            % Main GUI window
            hFig = figure('Name', 'Marble Golf Control Panel', ...
                        'NumberTitle', 'off', ...
                        'Position', [300, 300, 400, 500], ...
                        'MenuBar', 'none', ...
                        'ToolBar', 'none', ...
                        'NextPlot', 'new'); % Prevents plots from overwriting the GUI
                    
            % Add components (buttons, sliders, etc.)
            uicontrol('Style', 'text', 'Position', [50, 450, 300, 30], 'String', 'PlanarRobot8 Controller', 'FontSize', 12);
        
            % Button to initialize the robot
            uicontrol('Style', 'pushbutton', 'Position', [100, 400, 100, 30], ...
                    'String', 'Initialize Robot', ...
                    'Callback', @initializeRobot);
                            
            % Button to execute a function
            uicontrol('Style', 'pushbutton', 'Position', [100, 300, 100, 30], ...
                    'String', 'Retrieve Ball', ...
                    'Callback', @Retrieve);

            uicontrol('Style', 'pushbutton', 'Position', [100, 350, 100, 30], ...
                   'String', 'Roll Ball', ...
                    'Callback', @MarbleRoll);
 
            % Button to initialize the robot
            uicontrol('Style', 'pushbutton', 'Position', [250, 350, 100, 30], ...
                    'String', 'Emergency Stop', ...
                    'Callback', @togglePause);
        
            uicontrol('Style', 'text', 'Position', [10, 250, 200, 20], 'String', 'Current Position: ');
            hPositionText = uicontrol('Style', 'text', 'Position', [10, 220, 200, 20], 'String', 'X: 0, Y: 0, Z: 0');
        
            % Initialize the end effector position in handles
            handles.currentPosition = currentPosition;
            guidata(hFig, handles); % Save handles
        
            % Directional movement controls (buttons)
            uicontrol('Style', 'pushbutton', 'String', 'Move Up', 'Position', [10, 170, 100, 30], ...
                'Callback', @(~, ~) updatePosition(0, 0, 0.1));
            uicontrol('Style', 'pushbutton', 'String', 'Move Down', 'Position', [10, 130, 100, 30], ...
                'Callback', @(~, ~) updatePosition(0, 0, -0.1));
            uicontrol('Style', 'pushbutton', 'String', 'Move Left', 'Position', [120, 130, 100, 30], ...
                'Callback', @(~, ~) updatePosition(-0.1, 0, 0));
            uicontrol('Style', 'pushbutton', 'String', 'Move Right', 'Position', [120, 170, 100, 30], ...
                'Callback', @(~, ~) updatePosition(0.1, 0, 0));
            uicontrol('Style', 'pushbutton', 'String', 'Move Forward', 'Position', [240, 170, 100, 30], ...
                'Callback', @(~, ~) updatePosition(0, 0.1, 0));
            uicontrol('Style', 'pushbutton', 'String', 'Move Backward', 'Position', [240, 130, 100, 30], ...
                'Callback', @(~, ~) updatePosition(0, -0.1, 0));
        
            % Function to update position and refresh GUI
            function updatePosition(deltaX, deltaY, deltaZ)
                handles = guidata(hFig); % Retrieve updated handles
                r = getappdata(hFig, 'RobotInstance'); % Retrieve robot instance
                
                if isempty(r)
                    disp('Robot not initialized!');
                    return;
                end
            
                % Get updated position using moveEndEffector
                newPosition = r.moveEndEffector(deltaX, deltaY, deltaZ);
                
                % Display the new position in the GUI
                set(hPositionText, 'String', sprintf('X: %.2f, Y: %.2f, Z: %.2f', newPosition));
                
                % Store the updated position in the handles
                handles.currentPosition = newPosition;
                guidata(hFig, handles); % Save updated handles
            end
            
            
        
        
            % Callback functions
            function initializeRobot(~, ~)
                userResponse = questdlg('Is the Emergency Stop connected?', ...
                                        'Arduino Connection', ...
                                        'Yes', 'No', 'Yes');
                if strcmp(userResponse, 'Yes')
                    % Check for available serial devices
                    availableDevices = instrhwinfo('serial');
                    if any(contains(availableDevices.Port, 'Arduino'))
                        r = MarbleGolf(); % Assuming it initializes the robot and the Arduino
                        setappdata(hFig, 'RobotInstance', r);
                        disp('Robot initialized with Arduino.');
                    else
                        errordlg('No Arduino device found on the specified port. Please check the connection.', ...
                                'Connection Error');
                    end
                else
                    r = MarbleGolf(); % Initialize the robot without Arduino
                    setappdata(hFig, 'RobotInstance', r);
                    disp('Robot initialized without Estop.');
                    msgbox('Be careful! No Physical eStop Available.', ...
                        'Warning', 'warn');
                end
            end
        
            function Retrieve(~, ~)
                r = getappdata(hFig, 'RobotInstance');
                if isempty(r)
                    disp('Robot not initialized!');
                    return;
                end
                r.playGame(); % Call animate function (change this)
                disp('Running function in PlanarRobot8');
            end

            function MarbleRoll(~, ~)
                r = getappdata(hFig, 'RobotInstance');
                if isempty(r)
                    disp('Robot not initialized!');
                    return;
                end
                r.animateMarble(); % Call animate function (change this)
                disp('Running function in PlanarRobot8');
            end

            function togglePause()
                eStopGui;
            end
            
        end
    end
end
 