%% Map Configuration
map = figure();
map_size = 100;
axis ([0 map_size 0 map_size]);
axis square;
hold on;
map.Visible = "off";

%% Robot Configuration

% Start Configuration
robotStart = [5, 5];
robotDimensons = [2, 2];

% Target Configuration
robotTarget = [95, 95];

% Helper Variables
robotSX0 = robotStart(1)-robotDimensons(1);
robotSX1 = robotStart(1)+robotDimensons(1);
robotSY0 = robotStart(2)-robotDimensons(2);
robotSY1 = robotStart(2)+robotDimensons(2);

robotTX0 = robotTarget(1)-robotDimensons(1);
robotTX1 = robotTarget(1)+robotDimensons(1);
robotTY0 = robotTarget(2)-robotDimensons(2);
robotTY1 = robotTarget(2)+robotDimensons(2);

% Bounds Testing (Robot Inital and Target Configuration valid and on Map)


% Checks Raw Robot Start Coordinates
if (robotStart(1) < 0 || robotStart(1) > map_size || robotStart(2) < 0 || robotStart(2) > map_size)
    error("Robot Start Position is not within the map bounds.");
end

% Checks Raw Robot Target Coordinates
if (robotStart(1) < 0 || robotStart(1) > map_size || robotStart(2) < 0 || robotStart(2) > map_size)
    error("Robot Target Position is not within the map bounds.");
end

% Checks Start Coordinates with Robot Dimensions
if (robotSX0 < 0 || robotSX0 > map_size || robotSX1 < 0 || robotSX1 > map_size || robotSY0 < 0 || robotSY0 > map_size || robotSY1 < 0 || robotSY1 > map_size)
    error("Robot Dimensions too large given starting position (Exceeds map bounds).");
end

% Checks Start Coordinates with Robot Dimensions
if (robotTX0 < 0 || robotTX0 > map_size || robotTX1 < 0 || robotTX1 > map_size || robotTY0 < 0 || robotTY0 > map_size || robotTY1 < 0 || robotTY1 > map_size)
    error("Robot Dimensions too large given target position (Exceeds map bounds).");
end

%% Create Robot
robotVerticies = [robotSX0 robotSY0; robotSX0 robotSY1; robotSX1 robotSY1; robotSX1 robotSY0];
robot = polyshape(robotVerticies);

% Plot Start and Target Locations
robotTargetVerticies = [robotTX0 robotTY0; robotTX0 robotTY1; robotTX1 robotTY1; robotTX1 robotTY0];
robotTargetPoly = polyshape(robotTargetVerticies);
plot(robot, "FaceColor", "Red");
plot(robotTargetPoly, "FaceColor", "Green");

% Remove Helper Variables
clear robotSX0;
clear robotSX1;
clear robotSY0;
clear robotSY1;
clear robotTX0;
clear robotTX1;
clear robotTY0;
clear robotTY1;
clear robotTargetVerticies;
clear robotTargetPoly;

%% Map Obstacles
mapObstacles = [];
mapObstacles = CreateObstacle(mapObstacles, [20 10; 20 40; 60 40; 60 10]);
mapObstacles = CreateObstacle(mapObstacles, [50 60; 50 80; 70 80; 70 60]);
mapObstacleCount = numel(mapObstacles);

% Place Obstacles on Map
plot(mapObstacles, "FaceColor", "Black");





