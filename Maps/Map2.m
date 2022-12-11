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
mapObstacles = CreateObstacle(mapObstacles, [0 12; 0 14; 4 14; 4 12]);
mapObstacles = CreateObstacle(mapObstacles, [1 5; 1 10; 3 10; 3 5]);
mapObstacles = CreateObstacle(mapObstacles, [1 1; 1 3; 7 3; 7 1]);
mapObstacles = CreateObstacle(mapObstacles, [5 5; 5 10; 7 10; 7 5]);
mapObstacles = CreateObstacle(mapObstacles, [6 11; 6 16; 8 16; 8 11]);
mapObstacles = CreateObstacle(mapObstacles, [9 10; 9 13; 14 13; 14 10]);
mapObstacles = CreateObstacle(mapObstacles, [15 13; 15 16; 16 16; 16 13]);
mapObstacles = CreateObstacle(mapObstacles, [9 2; 9 7; 12 7; 12 2]);
mapObstacles = CreateObstacle(mapObstacles, [14 2; 14 8; 16 8; 16 2]);
mapObstacleCount = numel(mapObstacles);

% Place Obstacles on Map
plot(mapObstacles, "FaceColor", "Black");
grid on;
