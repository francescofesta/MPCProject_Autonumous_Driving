% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.8 (R2020a) and Automated Driving Toolbox 3.1 (R2020a).
% Generated on: 10-Jun-2020 11:58:25

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [0 20 0;
    50 20 0];
laneSpecification = lanespec(2, 'Width', 7.425);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [5.3 16.1 0]);

% Add the non-ego actors
actor(scenario, ...
    'ClassID', 5, ...
    'Length', 2.4, ...
    'Width', 0.76, ...
    'Height', 0.8, ...
    'Position', [17.9 16.1 0], ...
    'PlotColor', [166 166 166] / 255);

