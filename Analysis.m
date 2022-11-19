% Main Entry Point
%% Clear all previous configurations
clc;
clear;
close;

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
    warning("Only 1 Map available (Automatically Selecting)")
    mapSelection = 1;
else
    error("No Maps")
end

% Chooses Selected Map and Then Executes
selectedMap = AvailableMaps(mapSelection).name;

% Running of Map Populates following Key Variables:
    % Robot Start (Array)
    % Robot Target (Array)
    % Robot Dimensions (Array)
    % Robot (Polygon)
    % Robot Vertices (Array) - *Note to self: Potentially Clear*
    % Map Obstacles (Array of Polygons)
    % Map Obstacle Count (Number)
    % Map Size (Number)
    % Map (Figure)
run(selectedMap);


%% Get Algorithms
AvailableAlgorithms = dir(fullfile("Algorithms/", "*m"));
numberOfAlgorithms = numel(AvailableAlgorithms);

% Select Algorithim
if (numberOfAlgorithms > 1)
    for algIndex = 1:numberOfAlgorithms
        disp(algIndex + ") " + AvailableAlgorithms(algIndex).name);
    end
    algSelection = input("Please choose a map: ");
elseif (numberOfAlgorithms == 1)
    warning("Only 1 Map available (Automatically Selecting)")
    algSelection = 1;
else
    error("No Algorithms")
end

selectedAlgorithm = AvailableAlgorithms(algSelection).name;

%% Basic Configuration of Figure
figure(map);
title(append(extractBefore(selectedMap, strlength(selectedMap)-1), " - ", extractBefore(selectedAlgorithm, strlength(selectedAlgorithm)-1)));
map.Visible = "on";


%% Algorithm Execution Loop
% TODO
