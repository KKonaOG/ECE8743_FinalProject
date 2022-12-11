%% Map Configuration
map = figure();
map_size = 16;
axis ([0 map_size 0 map_size]);
yticks(0:1:map_size);
xticks(0:1:map_size);
axis square;
hold on;
map.Visible = "off";

%% Robot Configuration

% Start Configuration
robotStart = [0.5, 15.5];

% Target Configuration
robotTarget = [15.5, 0.5];

% Max Translation (Epsilon)
robotStepsize = map_size/20; % This value was back-calculated from a value of 5 for a map size of 100

% Goal Threshold
targetThreshold = map_size/22.22; % This value was back-calculated from a value of 3 for a map size of 100

% Bounds Testing (Robot Inital and Target Configuration valid and on Map)

% Checks Raw Robot Start Coordinates
if (robotStart(1) < 0 || robotStart(1) > map_size || robotStart(2) < 0 || robotStart(2) > map_size)
    error("Robot Start Position is not within the map bounds.");
end

% Checks Raw Robot Target Coordinates
if (robotStart(1) < 0 || robotStart(1) > map_size || robotStart(2) < 0 || robotStart(2) > map_size)
    error("Robot Target Position is not within the map bounds.");
end

% Plot Start and Target Locations
plot(robotStart(1), robotStart(2), "o");
plot(robotTarget(1), robotTarget(2), "*");

%% Map Obstacles
mapObstacles = [];
mapObstacles = CreateObstacle(mapObstacles, [0 0; 0 1; 4 1; 4 0]);
mapObstacles = CreateObstacle(mapObstacles, [0 4; 0 5; 4 5; 4 4]);
mapObstacles = CreateObstacle(mapObstacles, [0 8; 0 9; 4 9; 4 8]);
mapObstacles = CreateObstacle(mapObstacles, [0 13; 0 14; 4 14; 4 13]);
mapObstacles = CreateObstacle(mapObstacles, [3 2; 3 3; 5 3; 5 5; 6 5; 6 3; 8 3; 8 2]);
mapObstacles = CreateObstacle(mapObstacles, [3 6; 3 7; 8 7; 8 9; 9 9; 9 4; 8 4; 8 6;]);
mapObstacles = CreateObstacle(mapObstacles, [6 8; 6 10; 4 10; 4 12; 6 12; 6 14; 7 14; 7 12; 10 12; 10 10; 7 10; 7 8]);
mapObstacles = CreateObstacle(mapObstacles, [9 14; 9 15; 14 15; 14 14;]);
mapObstacles = CreateObstacle(mapObstacles, [12 10; 12 12; 16 12; 16 10;]);
mapObstacles = CreateObstacle(mapObstacles, [10 4; 10 9; 14 9; 14 8; 11 8; 11 5; 14 5; 14 4]);
mapObstacles = CreateObstacle(mapObstacles, [12 2; 12 3; 16 3; 16 2]);
mapObstacleCount = numel(mapObstacles);

% Place Obstacles on Map
plot(mapObstacles, "FaceColor", "Black");
grid on;
