function main_gui_simulation()
    % Create the main figure
    mainFig = figure('Name', 'Robot Simulation with GUI', ...
                     'NumberTitle', 'off', ...
                     'Position', [100, 100, 1000, 600]);

    % Create a panel for the GUI controls on the left side
    controlPanel = uipanel(mainFig, 'Title', 'Controls', ...
                           'Position', [0, 0, 0.25, 1]);  % 25% width for controls

    % Create a panel for the 3D simulation on the right side
    simPanel = uipanel(mainFig, 'Title', '3D Simulation', ...
                       'Position', [0.25, 0, 0.75, 1]);  % 75% width for simulation

    % Add a button in the control panel to initialize the robot
    uicontrol(controlPanel, 'Style', 'pushbutton', ...
              'String', 'Load Planar Robot', ...
              'Position', [10, 550, 120, 30], ...
              'Callback', @(src, event) load_planar_robot(simPanel));

    % Create an axes in the simPanel for 3D simulation
    simAxes = axes(simPanel, 'Position', [0.1, 0.1, 0.8, 0.8]);
    
    % Function to load the PlanarRobot8
    function load_planar_robot(panel)
    % Create the robot instance, passing the axes handle
    robot = PlanarRobot9(panel.Children(1));  % Updated from PlanarRobot8 to Gui
    end

end
