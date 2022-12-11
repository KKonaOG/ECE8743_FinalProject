%% Clear all previous configurations
clc;
clear;
close;
recycle on;

maps = dir(fullfile("Maps/", "*.m")); % Get all Maps
algorithms = dir("Algorithms/"); % Get all Algorithms
algorithms = algorithms(3:end); % Throw out the special directories

for run_i=1:5 % Modify this line to increase number of runs
    for map_i=1:numel(maps)
        disp("Map Selected: " + maps(map_i).name)
        selectedMap = maps(map_i).name; 
        for alg_i=1:numel(algorithms)
            disp("Algorithm Selected: " + algorithms(alg_i).name);
            selectedAlgorithm = algorithms(alg_i).name;
            runAlg(selectedAlgorithm, selectedMap, run_i)
        end
    end
end

% Helper function to execute Main.m in Analysis Mode
function runAlg(selectedAlgorithm, selectedMap, run_i)
    skipSelection = true;
    run(selectedMap);
    run("Main.m")
    save("DataLog\" + selectedAlgorithm + "_" + extractBefore(selectedMap, strlength(selectedMap)-1) + "_" + run_i + ".mat") % Saves Data for Review
end