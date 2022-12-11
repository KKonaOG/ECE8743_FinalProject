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
mapObstacles = CreateObstacle(mapObstacles, [2 8; 2 16; 4 16; 4 8;]);
mapObstacles = CreateObstacle(mapObstacles, [1 3; 1 6; 4 6; 4 3;]);
mapObstacles = CreateObstacle(mapObstacles, [5 1; 5 2; 9 2; 9 1;]);
mapObstacles = CreateObstacle(mapObstacles, [7 6; 7 10; 9 10; 9 6;]);
mapObstacles = CreateObstacle(mapObstacles, [8 13; 8 16; 9 16; 9 13;]);
mapObstacles = CreateObstacle(mapObstacles, [11 0; 11 8; 13 8; 13 0;]);
mapObstacles = CreateObstacle(mapObstacles, [12 11; 12 14; 15 14; 15 11;]);
mapObstacleCount = numel(mapObstacles);

% Place Obstacles on Map
plot(mapObstacles, "FaceColor", "Black");
grid on;