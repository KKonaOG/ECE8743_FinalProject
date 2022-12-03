%% WARNING: THIS IS A SCRIPT CALLED FROM Analysis.m, IT EXPECTS PRE-POPULATED VALUES FROM ITS EXECUTION
K = size(MVRRTTree, 2);
for k = 1:K
    %% Stage 1: RANDOM STATE
    Qrand = randi([0, map_size], [1, 2]);
    
    %% Stage 2: Find Nearest Neighbor
    K_nearest = size(MVRRTTree, 2); % Need a new size since it changes each loop
    Qnear = MVRRTTree(1).point; % Placeholder initial point as closest
    Qnear_index = 1; % Placeholder initial point as closest
    min_dist = pdist([MVRRTTree(1).point;Qrand]); % Placeholder initial point as closest
    for j = 1:K_nearest % Loop over entire checking for nearest point
        dist = pdist([MVRRTTree(j).point;Qrand]);
        if (dist < min_dist)
            min_dist = dist;
            Qnear_index = j;
            Qnear = MVRRTTree(j).point;
        end
    end

    %% Stage 3: Generate Input (Vector)
    % Calculate u vector
    u = [Qrand(1, 1) - Qnear(1, 1), Qrand(1, 2) - Qnear(1, 2)]; % Translation Vector
    if (min_dist > MVRRTTree_Epsilon) % If we move more than our Epsilon clamp it to Epsilon
        u_Dir = [u(1)/min_dist, u(2)/min_dist];
        u_Dir = MVRRTTree_Epsilon*u_Dir;
        u = u_Dir;
        min_dist = MVRRTTree_Epsilon;
    end

    %% Stage 4: Create Temporary New State (and do Collision Checks)
    Qnew = u + Qnear; % Create New "State"

    % Check for Collisions
    if (CheckCollision(Qnear, Qnew, mapObstacles))
        continue; % We had a collision move on
    end

    % Update Wp for Qnearest
    rule_1 = CheckDistanceRule(Qnear, Qrand, min_dist, mapObstacles, mapObstacleCount);
    min_wp = rule_1 * min_dist;

    %% Stage 5: Optimization of Parent Vertex Seleciton (MVRRT*) - Equivalent to Connect / Update
    % The Near procedure can be thought of as a generalization of
    % the nearest neighbor procedure in the sense that the former
    % returns a collection of vertices that are close to x, whereas the
    % latter returns only one such vertex that is the closest. Just like
    % the Nearest procedure, there are many ways to define the
    % Near procedure, each of which leads to different algorithmic
    % properties.
    Qneighborhood = [];
    Qneighborwp = [];
    for j = 1:K_nearest % Loop over entire tree checking for nearby points
        dist = pdist([MVRRTTree(j).point;Qnew]);
        rule_1 = CheckDistanceRule(MVRRTTree(j).point, Qnew, dist, mapObstacles, mapObstacleCount);
        wp = rule_1 * dist;
        % Line 7-12
        if (dist < y*((log(map_size)/map_size)^(1/2)))
            % Line 9
            if (~CheckCollision(MVRRTTree(j).point, Qnew, mapObstacles))
                % Lines 10-12
                if (Qnear_index ~= j && ((MVRRTTree(Qnear_index).cost + min_dist) > (MVRRTTree(j).cost + dist)) && ((MVRRTTree(Qnear_index).safety + min_wp) > (MVRRTTree(j).safety + wp)))
                    Qnear = MVRRTTree(j).point;
                    Qnear_index = j;
                    min_dist = dist;
                    min_wp = wp;
                end
            end
            Qneighborhood(size(Qneighborhood)+1) = j;
            Qneighborwp(size(Qneighborhood)+1) = wp;
        end
    end


    %% Stage 6: Add "Optimal" Path to Tree (Equivalent to Update)
    Qnew_index = size(MVRRTTree,2)+1;
    MVRRTTree(Qnew_index).point = Qnew;
    MVRRTTree(Qnew_index).distance = min_dist;
    MVRRTTree(Qnew_index).cost = min_dist + MVRRTTree(Qnear_index).cost;
    MVRRTTree(Qnew_index).safety = min_wp + MVRRTTree(Qnear_index).safety;
    MVRRTTree(Qnew_index).link = Qnear_index;
    MVRRTTree(Qnew_index).handle = line([Qnear(1) Qnew(1)], [Qnear(2) Qnew(2)]);
    
    %% Stage 7: Optimize Tree (Equivalent to Rewire)
    % Line 14-17
    for j = 1:size(Qneighborhood)
        % Line 15
        if ((Qneighborhood(j) ~= Qnear_index) && ~CheckCollision(MVRRTTree(Qneighborhood(j)).point, Qnew, mapObstacles))
            cost = min_dist + MVRRTTree(Qnear_index).cost + pdist([MVRRTTree(Qneighborhood(j)).point;Qnew]);
            safety = min_wp + MVRRTTree(Qnear_index).safety + Qneighborwp(j);

            if ((MVRRTTree(Qneighborhood(j)).cost > cost) && (MVRRTTree(Qneighborhood(j)).safety > safety))
                MVRRTTree(Qneighborhood(j)).cost = cost;
                MVRRTTree(Qneighborhood(j)).distance = pdist([MVRRTTree(Qneighborhood(j)).point;Qnew]);
                MVRRTTree(Qneighborhood(j)).link = size(MVRRTTree,2)+1;
                MVRRTTree(Qneighborhood(j)).safety = safety;

                 % Overhead for plotting
                delete(MVRRTTree(Qneighborhood(j)).handle);
                MVRRTTree(Qneighborhood(j)).handle = line([Qnew(1) MVRRTTree(Qneighborhood(j)).point(1)], [Qnew(2) MVRRTTree(Qneighborhood(j)).point(2)]);
            end
        end
    end

    if (MVRRTTree_Threshold > pdist([Qnew; robotTarget]))
        isDone = true;
        break;
    end
end

function [safety_factor] = CheckDistanceRule(S_near, S_prime, trajDist, Sobst, obstCount)
    % Rule - Distance to nearest obstacle < 5
    % Weight of Rule - 0 if dist > 5 | (5-dist)/5 if dist < 5
               
    closestDist = Inf;
    for n=1:obstCount
        % Generate Points Along Tragectory

        if (trajDist < 1)
            obst_dist = p_poly_dist(S_prime(1), S_prime(2), Sobst(n).Vertices(:,1), Sobst(n).Vertices(:,2));
            closestDist = min([closestDist obst_dist]);
        else
            u_traj_points_x = linspace(S_near(1), S_prime(1), 3); % Start Point, End Point, Mid Point
            u_traj_points_y = linspace(S_near(2), S_prime(2), 3);
            u_traj_points = [u_traj_points_x' u_traj_points_y'];
            for np = 1:size(u_traj_points, 1)
                obst_dist = p_poly_dist(u_traj_points(np, 1), u_traj_points(np, 2), Sobst(n).Vertices(:,1), Sobst(n).Vertices(:,2));
                closestDist = min([closestDist obst_dist]);
            end 
        end
    end

    if (closestDist > 5)
        safety_factor = 0;
    else
        % Calculate Weight
        safety_factor = (5-closestDist)/5;
    end
end
 