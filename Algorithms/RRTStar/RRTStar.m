%% WARNING: THIS IS A SCRIPT CALLED FROM Analysis.m, IT EXPECTS PRE-POPULATED VALUES FROM ITS EXECUTION
K = size(RRTStarTree, 2);
for k = 1:K
    %% Stage 1: RANDOM STATE
    Qrand = randi([0, map_size], [1, 2]);
    
    %% Stage 2: Find Nearest Neighbor
    K_nearest = size(RRTStarTree, 2); % Need a new size since it changes each loop
    Qnear = RRTStarTree(1).point; % Placeholder initial point as closest
    Qnear_index = 1; % Placeholder initial point as closest
    min_dist = pdist([RRTStarTree(1).point;Qrand]); % Placeholder initial point as closest
    for j = 1:K_nearest % Loop over entire checking for nearest point
        dist = pdist([RRTStarTree(j).point;Qrand]);
        if (dist < min_dist)
            min_dist = dist;
            Qnear_index = j;
            Qnear = RRTStarTree(j).point;
        end
    end

    %% Stage 3: Generate Input (Vector)
    % Calculate u vector
    u = [Qrand(1, 1) - Qnear(1, 1), Qrand(1, 2) - Qnear(1, 2)]; % Translation Vector
    if (min_dist > RRTStarTree_Epsilon) % If we move more than our Epsilon clamp it to Epsilon
        u_Dir = [u(1)/min_dist, u(2)/min_dist];
        u_Dir = RRTStarTree_Epsilon*u_Dir;
        u = u_Dir;
        min_dist = RRTStarTree_Epsilon;
    end

    %% Stage 4: Create Temporary New State (and do Collision Checks)
    Qnew = u + Qnear; % Create New "State"

    % Check for Collisions
    if (CheckCollision(Qnear, Qnew, mapObstacles))
        continue; % We had a collision move on
    end

    %% Stage 5: Optimization of Parent Vertex Seleciton (RRT*)
    % The Near procedure can be thought of as a generalization of
    % the nearest neighbor procedure in the sense that the former
    % returns a collection of vertices that are close to x, whereas the
    % latter returns only one such vertex that is the closest. Just like
    % the Nearest procedure, there are many ways to define the
    % Near procedure, each of which leads to different algorithmic
    % properties.

    % Used Neighborhood Method Described Here: https://www.youtube.com/watch?v=JM7kmWE8Gtc
    % Described as Bx,r in arxiv:1005.0416 (where r is an arbitrary value)
    % Neighbordhood is described as Step-Size*2 (RRTStarTree_Epsilon*2)
    % This value was selected purely as a random abstraction of the space
    Qneighborhood = [];
    for j = 1:K_nearest % Loop over entire tree checking for nearby points
        dist = pdist([RRTStarTree(j).point;Qnew]);
        % Line 7-12
        if (dist < RRTStarTree_Epsilon*2)
            % Line 9
            if (~CheckCollision(RRTStarTree(j).point, Qnew, mapObstacles))
                % Lines 10-12
                if (Qnear_index ~= j && ((RRTStarTree(Qnear_index).cost + min_dist) > (RRTStarTree(j).cost + dist)))
                    Qnear = RRTStarTree(j).point;
                    Qnear_index = j;
                    min_dist = dist;
                end
            end
            Qneighborhood(size(Qneighborhood)+1) = j;
        end
    end

    %% Stage 6: Add "Optimal" Path to Tree
    Qnew_index = size(RRTStarTree,2)+1;
    RRTStarTree(Qnew_index).point = Qnew;
    RRTStarTree(Qnew_index).distance = min_dist;
    RRTStarTree(Qnew_index).cost = min_dist + RRTStarTree(Qnear_index).cost;
    RRTStarTree(Qnew_index).link = Qnear_index;

    %% Plot Lines (over head for plotting)
    RRTStarTree(Qnew_index).handle = line([Qnear(1) Qnew(1)], [Qnear(2) Qnew(2)]);
    
    %% Stage 7: Optimize Tree
    % Line 14-17
    for j = 1:size(Qneighborhood)
        % Line 15
        if ((Qneighborhood(j) ~= Qnear_index) && ~CheckCollision(RRTStarTree(Qneighborhood(j)).point, Qnew, mapObstacles))
            if (RRTStarTree(Qneighborhood(j)).cost > (min_dist + RRTStarTree(Qnear_index).cost + pdist([RRTStarTree(Qneighborhood(j)).point;Qnew])))
                RRTStarTree(Qneighborhood(j)).cost = min_dist + RRTStarTree(Qnear_index).cost + pdist([RRTStarTree(Qneighborhood(j)).point;Qnew]);
                RRTStarTree(Qneighborhood(j)).distance = pdist([RRTStarTree(Qneighborhood(j)).point;Qnew]);
                RRTStarTree(Qneighborhood(j)).link = size(RRTStarTree,2)+1;

                % Overhead for plotting
                delete(RRTStarTree(Qneighborhood(j)).handle);
                RRTStarTree(Qneighborhood(j)).handle = line([Qnew(1) RRTStarTree(Qneighborhood(j)).point(1)], [Qnew(2) RRTStarTree(Qneighborhood(j)).point(2)]);
            end
        end
    end


    if (RRTStarTree_Threshold > pdist([Qnew; robotTarget]))
        isDone = true;
        break;
    end
end
 