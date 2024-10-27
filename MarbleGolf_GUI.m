function MarbleGolf_GUI
    % Create a dummy figure and immediately close it to clear the figure state
    figure;
    close;
    
    % Main GUI window
    hFig = figure('Name', 'Marble Golf Control Panel', ...
                  'NumberTitle', 'off', ...
                  'Position', [300, 300, 400, 500], ...
                  'MenuBar', 'none', ...
                  'ToolBar', 'none', ...
                  'NextPlot', 'new'); % Prevents plots from overwriting the GUI
              
    % Add components (buttons, sliders, etc.)
    uicontrol('Style', 'text', 'Position', [50, 450, 300, 30], 'String', 'PlanarRobot8 Controller', 'FontSize', 12);

    % Other UI components
    % Button to initialize the robot
    uicontrol('Style', 'pushbutton', 'Position', [50, 400, 100, 30], ...
              'String', 'Initialize Robot', ...
              'Callback', @initializeRobot);
          
    % Slider for controlling a joint angle (example for Joint 1)
    uicontrol('Style', 'text', 'Position', [50, 350, 100, 20], 'String', 'Joint 1 Angle');
    hSlider1 = uicontrol('Style', 'slider', 'Position', [150, 350, 200, 20], ...
                         'Min', -180, 'Max', 180, 'Value', 0, ...
                         'Callback', @(src, event) setJointAngle(1, src.Value));
                     
    % Button to execute a function
    uicontrol('Style', 'pushbutton', 'Position', [50, 300, 100, 30], ...
              'String', 'Retrieve Ball', ...
              'Callback', @Retrieve);

    % Store GUI data
    data.hSlider1 = hSlider1;
    guidata(hFig, data);

    % Callback functions
    function initializeRobot(~, ~)
        r = MarbleGolf(); % Assuming it initializes when called directly
        setappdata(hFig, 'RobotInstance', r);
        disp('Robot initialized');
    end

    function setJointAngle(joint, angle)
        r = getappdata(hFig, 'RobotInstance');
        if isempty(r)
            disp('Robot not initialized!');
            return;
        end
        fprintf('Setting Joint %d to angle %.2f\n', joint, angle);
        % r.setJointAngle(joint, angle); % Example function call if available
    end

    function Retrieve(~, ~)
        r = getappdata(hFig, 'RobotInstance');
        if isempty(r)
            disp('Robot not initialized!');
            return;
        end
        r.animateRobot;
        % Example: r.moveRobot(); % Call specific function here
        disp('Running function in PlanarRobot8');
    end
end
