% Main Entry Point
%% Clear all previous configurations
if (~exist("skipSelection", "var"))
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

    %% Show Map if main program is executed directly
    map.Visible = "on";
end
%% Basic Configuration of Figure
title(append(extractBefore(selectedMap, strlength(selectedMap)-1), " - ", sprintf("%s", selectedAlgorithm)));

%% Minimum "Resolution" (Iterations)
minIter = 500;
isDone = false; % Lets us keep running if we haven't found a solution after minIter;
dataRefreshCount = 100;

%% Algorithm Execution Loop
if (selectedAlgorithm == "RRT")
    % Metric Preperations
    delete(sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm));
    totalDuration = 0;
    RRTTree_times = [];
    
    % RRT Configuration Variables
    RRTTree_Epsilon = robotStepsize; % Max allowed travel distance
    RRTTree_Threshold = targetThreshold; % Radius from target that a point must be to consider the target as reached
    RRTTree_Iterations = 0;

    % Goal Point
    RRTTree_Goals = [];

    % Plot Threshold Goal Region
    viscircles(robotTarget, RRTTree_Threshold, "LineWidth", 0.5, "Color", "Red");

    % Tree Initialization
    RRTTree = struct();
    RRTTree(1).point = robotStart;
    RRTTree(1).distance = 0;
    RRTTree(1).cost = 0;
    RRTTree(1).link = 0;
    
    while (RRTTree_Iterations<minIter || ~isDone)
        tic;
        run("Algorithms\RRT\RRT.m");
        iter_end = toc;
        RRTTree_times = [RRTTree_times;iter_end];

        % Metric Related Work (not considered in execution time)
        totalDuration = totalDuration + iter_end;
        RRTTree_Iterations = RRTTree_Iterations + 1;
        subtitle(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("Iteration #%d Duration: %f s\n", RRTTree_Iterations, iter_end), "Total Iterations: ", int2str(RRTTree_Iterations)));

        % Logging Output (and saving images)
        if (mod(RRTTree_Iterations, dataRefreshCount) == 0)
            exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);
            disp(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("RRT Iteration #%d Duration: %f s\n", RRTTree_Iterations, iter_end), "Total Iterations: ", int2str(RRTTree_Iterations)));
            disp("Tree Size: " + size(RRTTree, 2));
            disp("Goals Found: " + numel(RRTTree_Goals))
        end
    end
    
    FinalGoals = RRTTree_Goals;
    FinalTree = RRTTree;
end

if (selectedAlgorithm == "RRTStar")
    % Metric Preperations
    delete(sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm));
    totalDuration = 0;
    
    % RRT Configuration Variables
    RRTStarTree_Epsilon = robotStepsize; % Max allowed travel distance
    RRTStarTree_Threshold = targetThreshold; % Radius from target that a point must be to consider the target as reached
    RRTStarTree_Iterations = 0;
    RRTStarTree_times = [];

    % Goal Points
    RRTStarTree_Goals = [];

    % Plot Threshold Goal Region
    viscircles(robotTarget, RRTStarTree_Threshold, "LineWidth", 0.5, "Color", "Red");

    % Tree Initialization
    RRTStarTree = struct();
    RRTStarTree(1).point = robotStart;
    RRTStarTree(1).distance = 0;
    RRTStarTree(1).cost = 0;
    RRTStarTree(1).link = 0;
    RRTStarTree(1).handle = NaN;
    
    while (RRTStarTree_Iterations<minIter || ~isDone)
        tic;
        run("Algorithms\RRTStar\RRTStar.m");
        iter_end = toc;
        RRTStarTree_times = [RRTStarTree_times;iter_end];

        % Metric Related Work (not considered in execution time)
        totalDuration = totalDuration + iter_end;
        RRTStarTree_Iterations = RRTStarTree_Iterations + 1;

        % Logging Output (and saving images)
        if (mod(RRTStarTree_Iterations, dataRefreshCount) == 0)
            subtitle(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("Iteration #%d Duration: %f s\n", RRTStarTree_Iterations, iter_end), "Total Iterations: ", int2str(RRTStarTree_Iterations)));
            exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);
            disp(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("RRT* Iteration #%d Duration: %f s\n", RRTStarTree_Iterations, iter_end), "Total Iterations: ", int2str(RRTStarTree_Iterations)));
            disp("Tree Size: " + size(RRTStarTree, 2));
            disp("Goals Found: " + numel(RRTStarTree_Goals))
        end
    end

    FinalGoals = RRTStarTree_Goals;
    FinalTree = RRTStarTree;
end

if (selectedAlgorithm == "MVRRT")
    % Metric Preperations
    delete(sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm));
    totalDuration = 0;
    
    % MVRRT Configuration Variables
    MVRRTTree_Epsilon = robotStepsize;
    MVRRTTree_Threshold = targetThreshold; % Radius from target that a point must be to consider the target as reached
    MVRRTTree_Iterations = 0;
    MVRRTTree_times = [];

    % Plot Threshold Goal Region
    viscircles(robotTarget, MVRRTTree_Threshold, "LineWidth", 0.5, "Color", "Red");

    % From Arxiv:1305.1102
    % In this proof, γ > (2*(2 + 1/d)^(1/d))*((µ(S)/ζd)^(1/d)), where 
    % µ(S) is the Lebesgue measure of the set S and ζd is the
    % volume of the unit ball of dimensionality d.
    y = (2*(2 + (1/2))^(1/2)) * (((map_size^2)/(pi*((MVRRTTree_Epsilon*2)^2)))^(1/2))+1;

    % Tree Initlization
    MVRRTTree = struct();
    MVRRTTree(1).point = robotStart;
    MVRRTTree(1).distance = 0;
    MVRRTTree(1).cost = 0;
    MVRRTTree(1).safety = 0;
    MVRRTTree(1).link = 0;
    MVRRTTree(1).handle = NaN;
    
    % Goal Points
    MVRRTTree_Goals = [];

    while (MVRRTTree_Iterations < minIter || ~isDone)
        tic;
        run("Algorithms\MVRRT\MVRRT.m");
        iter_end = toc;
        MVRRTTree_times = [MVRRTTree_times;iter_end];

        % Metrics
        totalDuration = totalDuration + iter_end;
        MVRRTTree_Iterations = MVRRTTree_Iterations + 1;
        subtitle(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("Iteration #%d Duration: %f s\n", MVRRTTree_Iterations, iter_end), "Total Iterations: ", int2str(MVRRTTree_Iterations)));
        
        % Logging Output (and saving images)
        if (mod(MVRRTTree_Iterations, dataRefreshCount) == 0)
            exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);
            disp(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("MVRRT Iteration #%d Duration: %f s\n", MVRRTTree_Iterations, iter_end), "Total Iterations: ", int2str(MVRRTTree_Iterations)));
            disp("Tree Size: " + size(MVRRTTree, 2));
            disp("Goals Found: " + numel(MVRRTTree_Goals))
        end
    end
    FinalGoals = MVRRTTree_Goals;
    FinalTree = MVRRTTree;
end

if (selectedAlgorithm == "SVGRRT")
    % Metric Preperations
    delete(sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm));
    totalDuration = 0;
    
    % MVRRT Configuration Variables
    SVGRRTTree_Epsilon = robotStepsize;
    SVGRRTTree_Threshold = targetThreshold; % Radius from target that a point must be to consider the target as reached
    SVGRRTTree_Iterations = 0;
    SVGRRTTree_Times = [];

    % Goal Points
    SVGRRTTree_Goals = [];

    % Plot Threshold Goal Region
    viscircles(robotTarget, SVGRRTTree_Threshold, "LineWidth", 0.5, "Color", "Red");

    % Tree Initlization
    SVGRRTTree = struct();
    SVGRRTTree(1).point = robotStart;
    SVGRRTTree(1).distance = 0;
    SVGRRTTree(1).cost = 0;
    SVGRRTTree(1).link = 0;

    while (SVGRRTTree_Iterations<minIter || ~isDone)
            tic;
            run("Algorithms\SVGRRT\SVGRRT.m");
            iter_end = toc;
            SVGRRTTree_Times = [SVGRRTTree_Times iter_end];

            % Metrics
            totalDuration = totalDuration + iter_end;
            SVGRRTTree_Iterations = SVGRRTTree_Iterations + 1;
            subtitle(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("Iteration #%d Duration: %f s\n", SVGRRTTree_Iterations, iter_end), "Total Iterations: ", int2str(SVGRRTTree_Iterations)));
            
            % Logging Output (and saving images)
            if (mod(SVGRRTTree_Iterations, dataRefreshCount) == 0)
                exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);
                disp(append(sprintf("Delta Time: %f s\n", totalDuration), sprintf("SVGRRT Iteration #%d Duration: %f s\n", SVGRRTTree_Iterations, iter_end), "Total Iterations: ", int2str(SVGRRTTree_Iterations)));
                disp("Tree Size: " + size(SVGRRTTree, 2));
                disp("Goals Found: " + numel(SVGRRTTree_Goals))
            end
    end
    FinalGoals = SVGRRTTree_Goals;
    FinalTree = SVGRRTTree;
end

% Determines Best Goal Point
BestGoal = FinalGoals(1);
for bgI=2:numel(FinalGoals)
    if (FinalGoals(bgI).cost < BestGoal.cost)
        BestGoal = FinalGoals(bgI);
    end
end

% Show Best Goal Path
ClosestObstacleInteraction = Inf; % Equivalent to Safety
GoalPoint = BestGoal;
while (GoalPoint.link ~= 0)
    h = line([GoalPoint.point(1) FinalTree(GoalPoint.link).point(1)], [GoalPoint.point(2) FinalTree(GoalPoint.link).point(2)]);
    h.Color = "Red";
    exportgraphics(gcf, sprintf("Graphics/%s_Map_%s_Algorithm.gif", extractBefore(selectedMap, strlength(selectedMap)-1), selectedAlgorithm), 'Append', true);

    xtraverse = linspace(GoalPoint.point(1), FinalTree(GoalPoint.link).point(1), map_size^2);
    ytraverse = linspace(GoalPoint.point(2), FinalTree(GoalPoint.link).point(2), map_size^2);
    

    % Determines Closest Obstacle Distance for an individual segment
    closest_obst_dist = Inf;
    for sIdx=1:map_size^2
        point = [xtraverse(sIdx) ytraverse(sIdx)];
        for oIdx=1:numel(mapObstacles)
            curr_obst_dist = abs(p_poly_dist(xtraverse(sIdx), ytraverse(sIdx), mapObstacles(oIdx).Vertices(:,1), mapObstacles(oIdx).Vertices(:,2)));
            closest_obst_dist = min(curr_obst_dist, closest_obst_dist);
        end
    end
    ClosestObstacleInteraction = min(closest_obst_dist, ClosestObstacleInteraction);
    GoalPoint = FinalTree(GoalPoint.link);
end
