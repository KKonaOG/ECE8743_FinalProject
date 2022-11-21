% Main Entry Point
%% Clear all previous configurations
clc;
clear;
close;
recycle on;

%% Get Maps
AvailableMaps = dir(fullfile("Maps/", "*.m"));
numberOfMaps = numel(AvailableMaps);

% Select Map
if (numberOfMaps > 1)
    for mapIndex = 1:numberOfMaps
        disp(mapIndex + ") " + AvailableMaps(mapIndex).name);
    end
    mapSelection = input("Please choose a map: ");
elseif (numberOfMaps == 1)
    warning("Only 1 Map available (Automatically Selecting)");
    mapSelection = 1;
else
    error("No Maps")
end

% Chooses Selected Map and Then Executes
selectedMap = AvailableMaps(mapSelection).name;

% Running of Map exposes the following variables:
    % Robot Start (1 x 2 Array)
    % Robot Target (1 x 2 Array)
    % Robot Dimensions (1 x 2 Array)
    % Robot (Polygon)
    % Robot Vertices (Several Point Numbers) - *Note to self: Potentially Clear*
    % Map Obstacles (Array of Polygons)
    % Map Obstacle Count (Number)
    % Map Size (Number)
    % Map (Figure)
run(selectedMap);


%% Get Algorithms
AvailableAlgorithms = dir("Algorithms/");
AvailableAlgorithms = AvailableAlgorithms(3:end);
numberOfAlgorithms = numel(AvailableAlgorithms);

% Select Algorithim
if (numberOfAlgorithms > 1)
    for algIndex = 1:numberOfAlgorithms
        disp(algIndex + ") " + AvailableAlgorithms(algIndex).name);
    end
    algSelection = input("Please choose an algorithm: ");
elseif (numberOfAlgorithms == 1)
    warning("Only 1 Algorithm available (Automatically Selecting)");
    algSelection = 1;
else
    error("No Algorithms")
end

selectedAlgorithm = AvailableAlgorithms(algSelection).name;

%% Basic Configuration of Figure
figure(map);
title(append(extractBefore(selectedMap, strlength(selectedMap)-1), " - ", selectedAlgorithm));
map.Visible = "on";


%% Algorithm Execution Loop
if (selectedAlgorithm == "RRT")
    % Metric Preperations
    delete(sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm));
    totalDuration = 0;
    
    % RRT Configuration Variables
    RRTTree_Epsilon = 5; % Max allowed travel distance
    RRTTree_Threshold = 3; % Radius from target that a point must be to consider the target as reached
    RRTTree_Iterations = 0;

    % Tree Initialization
    RRTTree = struct();
    RRTTree(1).point = robotStart;
    RRTTree(1).distance = 0;
    RRTTree(1).link = 0;
    
    % isDone is tracked as a flag to stop execution
    isDone = false;

    while ~isDone
        tic;
        run("Algorithms\RRT\RRT.m");
        iter_end = toc;

        % Metric Related Work (not considered in execution time)
        totalDuration = totalDuration + iter_end;
        RRTTree_Iterations = RRTTree_Iterations + 1;
        subtitle(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("Iteration #%d Duration: %f s\n", RRTTree_Iterations, iter_end), "Total Iterations: ", int2str(RRTTree_Iterations)));
        exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);
    end
end

if (selectedAlgorithm == "RRTStar")
end

if (selectedAlgorithm == "MVRRT")
end

if (selectedAlgorithm == "SVGRRT")
end





