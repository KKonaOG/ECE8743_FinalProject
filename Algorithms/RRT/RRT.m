%% WARNING: THIS IS A SCRIPT CALLED FROM Analysis.m, IT EXPECTS PRE-POPULATED VALUES FROM ITS EXECUTION
K = size(RRTTree, 2);
for k = 1:K
    %% Stage 1: RANDOM STATE
    Qrand = randi([0, map_size], [1, 2]);
    
    %% Stage 2: Find Nearest Neighbor
    K_nearest = size(RRTTree, 2); % Need a new size since it changes each loop
    Qnear = RRTTree(1).point; % Placeholder initial point as closest
    Qnear_index = 1; % Placeholder initial point as closest
    min_dist = pdist([RRTTree(1).point;Qrand]); % Placeholder initial point as closest
    for j = 1:K_nearest % Loop over entire checking for nearest point
        dist = pdist([RRTTree(j).point;Qrand]);
        if (dist < min_dist)
            min_dist = dist;
            Qnear_index = j;
            Qnear = RRTTree(j).point;
        end
    end

    %% Stage 3: Generate Input (Vector)
    % Calculate u vector
    u = [Qrand(1, 1) - Qnear(1, 1), Qrand(1, 2) - Qnear(1, 2)]; % Translation Vector
    if (min_dist > RRTTree_Epsilon) % If we move more than our Epsilon clamp it to Epsilon
        u_Dir = [u(1)/min_dist, u(2)/min_dist];
        u_Dir = RRTTree_Epsilon*u_Dir;
        u = u_Dir;
        min_dist = RRTTree_Epsilon;
    end


    %% Stage 4: Create New State (and do Collision Checks)
    Qnew = u + Qnear; % Create New "State"

    % Check for Collisions
    if (CheckCollision(Qnear, Qnew, mapObstacles))
        continue; % We had a collision move on
    end

    %% Stage 5: Add to Tree
    Qnew_index = size(RRTTree,2)+1;
    RRTTree(Qnew_index).point = Qnew;
    RRTTree(Qnew_index).distance = min_dist;
    RRTTree(Qnew_index).link = Qnear_index;

    %% Plot Lines
    line([Qnear(1) Qnew(1)], [Qnear(2) Qnew(2)]);
    if (RRTTree_Threshold > pdist([Qnew; robotTarget]))
        isDone = true;
        break;
    end
end
 