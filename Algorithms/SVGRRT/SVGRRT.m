%% WARNING: THIS IS A SCRIPT CALLED FROM Main.m, IT EXPECTS PRE-POPULATED VALUES FROM ITS EXECUTION
didVis = false;
%% Stage 1: RANDOM STATE
if (SVGRRTTree_Iterations >= minIter)
    Qrand = robotTarget;
else
    Qrand = randi([0, map_size], [1, 2]);
end

%% Stage 2: Find Nearest Neighbor
K_nearest = size(SVGRRTTree, 2); % Need a new size since it changes each loop
Qnear = SVGRRTTree(1).point; % Placeholder initial point as closest
Qnear_index = 1; % Placeholder initial point as closest
min_dist = pdist([SVGRRTTree(1).point;Qrand]); % Placeholder initial point as closest
for j = 1:K_nearest % Loop over entire checking for nearest point
    dist = pdist([SVGRRTTree(j).point;Qrand]);
    if (dist < min_dist)
        min_dist = dist;
        Qnear_index = j;
        Qnear = SVGRRTTree(j).point;
    end
end

%% Stage 3: Generate Input (Vector)
% Calculate u vector
u = [Qrand(1, 1) - Qnear(1, 1), Qrand(1, 2) - Qnear(1, 2)]; % Translation Vector
if (min_dist > SVGRRTTree_Epsilon) % If we move more than our Epsilon clamp it to Epsilon
    u_Dir = [u(1)/min_dist, u(2)/min_dist];
    u_Dir = SVGRRTTree_Epsilon*u_Dir;
    u = u_Dir;
    min_dist = SVGRRTTree_Epsilon;
end

%% Stage 4: Create New State (and do Collision Checks)
Qnew = u + Qnear; % Create New "State"

% Check for Collisions
if (~CheckCollision(Qnear, Qnew, mapObstacles, map_size))
    % Visibility Optimization for Nearby Obstacles
    
    % Find the closest obstacle
    min_obst_dist = Inf;
    min_obst = NaN; % Closest Obstacle
    min_obst_point = NaN; % Closest Obstacle Vertex
    min_obst_idx = NaN; % Closest Obstacle Index
    min_obst_edge_vert = NaN; % Other vertex on the edge
    for obsInd = 1:mapObstacleCount
        obstacle = mapObstacles(obsInd);
        for vIdx=1:size(obstacle.Vertices, 1)
            vDist = pdist([obstacle.Vertices(vIdx, :); Qnew]);
            if (vDist < min_obst_dist) 
                min_obst_dist = vDist;
                min_obst = obstacle;
                min_obst_idx = obsInd;

                % Handles the determined vertex being at the end of the
                % obstacles vertices (connects to 1 if it is at end)
                if (vIdx == size(obstacle.Vertices, 1))
                    min_obst_egde_vert = 1;
                else
                    min_obst_edge_vert = vIdx+1;
                end
                min_obst_point = obstacle.Vertices(vIdx, :);
            end
        end
    end
    
    % Activates based off proximity to the vertex
    if (min_obst_dist < SVGRRTTree_Epsilon/4)
        % Determine Edge Vertices
        Qnew = min_obst_point;
        min_dist = min_obst_dist;
    
        % Generate Visibility Graph to other Obst and Goal (uses only one
        % edge on closest obstacle)
        % Prepopulate with data discovered above
        VG = struct();
        VG(1).node = min_obst_point;
        VG(2).node = min_obst.Vertices(min_obst_edge_vert, :);
        for obsInd = 1:mapObstacleCount
            % Don't re-store min_obst
            if (obsInd == min_obst_idx)
                continue;
            end

            % Add vertices to VG
            obstacle = mapObstacles(obsInd);
            for vertIndx2=1:size(obstacle.Vertices, 1)
                VG(size(VG, 2)+1).node = [obstacle.Vertices(vertIndx2,1), obstacle.Vertices(vertIndx2,2)];
            end
        end
    
        VG(end+1).node = robotTarget; % Add goal
    
        % Add the Vertices to a graph if they do not collide
        G = graph();
        for vgElemIdx=1:size(VG, 2)
            for vgElemIdx2=1:size(VG, 2)
                % Don't process self
                if (vgElemIdx==vgElemIdx2)
                    continue;
                end
    
                % Check for Edge Traversal Collisions
                didCollide = false;
                traversalX = linspace(VG(vgElemIdx).node(1), VG(vgElemIdx2).node(1), min(map_size^2, 500)); % Resolution is related to map_size (Caps off at 500)
                traversalY = linspace(VG(vgElemIdx).node(2), VG(vgElemIdx2).node(2), min(map_size^2, 500)); % Resolution is related to map_size (Capps off at 500)
            
                for obsInd2 = 1:mapObstacleCount
                    obstacle = mapObstacles(obsInd2);
                    
                    % Checks traversals to make sure they are not outside
                    % the map
                    if (any(traversalX<=0)||any(traversalX>=map_size))
                        didCollide = true;
                    end
    
                    if (any(traversalY<=0)||any(traversalY>=map_size))
                        didCollide = true;
                    end
                    
                    % Collides if there is point in (exlcudes the ones on
                    % the polygon)
                    [in, on] = inpolygon(traversalX(1:end-1), traversalY(1:end-1), obstacle.Vertices(:, 1), obstacle.Vertices(:, 2));
                    if max(xor(in,on))
                        didCollide = true;
                        break;
                    end
                end
    
                if (didCollide)
                    continue
                end

                G = G.addedge(vgElemIdx, vgElemIdx2, pdist([VG(vgElemIdx).node;VG(vgElemIdx2).node])); % Add edge to graph
            end
        end

        % Calculate the shortest path to goal
        path = G.shortestpath(1, size(VG, 2));

        % If we found a valid path (can happen in some instances)
        if (~(numel(path) < 2))
             % This will be added as a 2nd-step, repeat 3rd/4th Stage
            Qvis = VG(path(2)).node;
            Qvis_vert = VG(path(1)).node;
        
            % Calculate u vector
            u = [Qvis(1, 1) - Qvis_vert(1, 1), Qvis(1, 2) - Qvis_vert(1, 2)]; % Translation Vector
            Qvis_dist = pdist([Qvis;Qvis_vert]);
            if (Qvis_dist > SVGRRTTree_Epsilon) % If we move more than our Epsilon clamp it to Epsilon
                u_Dir = [u(1)/Qvis_dist, u(2)/Qvis_dist];
                u_Dir = (SVGRRTTree_Epsilon)*u_Dir;
                u = u_Dir;
                Qvis_dist = SVGRRTTree_Epsilon;
            end
            
            Qvis = u + Qvis_vert; % Create New "State"
        
            didVis = true;
        end
    end

    %% Stage 5: Add to Tree

    % Plot Qnew (and translation to Qnear(which is either a new point or an
    % obstacle vertex)
    no = line([Qnear(1) Qnew(1)], [Qnear(2) Qnew(2)]);
    
    % If we did a visibility graph optimization then we are adding a few
    % more points
    if (didVis) 
        % Add obstacle vertex to the graph
        Qnew3_index = size(SVGRRTTree,2)+1;
        SVGRRTTree(Qnew3_index).point = Qvis_vert;
        SVGRRTTree(Qnew3_index).distance = pdist([Qvis_vert;Qnew]);
        SVGRRTTree(Qnew3_index).cost = min_dist + SVGRRTTree(Qnear_index).cost + pdist([Qvis_vert;Qnew]);
        SVGRRTTree(Qnew3_index).link = Qnear_index;
        
        % Plot the transition from Qnear to obstacle vertice
        no = line([Qnear(1) Qvis_vert(1)], [Qnear(2) Qvis_vert(2)]);
        no.Color = "Red";
        
        % Checks to see if somehow this the goal (shouldn't happen)
        if (SVGRRTTree_Threshold > pdist([Qvis_vert; robotTarget]))
            isDone = true;
            SVGRRTTree_Goals = [SVGRRTTree_Goals; SVGRRTTree(Qnew3_index)];
        end
        
        % Adds the step we found using the visibility graph
        Qnew2_index = size(SVGRRTTree,2)+1;
        SVGRRTTree(Qnew2_index).point = Qvis;
        SVGRRTTree(Qnew2_index).distance = Qvis_dist;
        SVGRRTTree(Qnew2_index).cost = min_dist + SVGRRTTree(Qnear_index).cost + Qvis_dist;
        SVGRRTTree(Qnew2_index).link = Qnew3_index;
        
        no = line([Qvis_vert(1) Qvis(1)], [Qvis_vert(2) Qvis(2)]);
        no.Color = "Blue";

         % Checks to see if we have reached the goal
        if (SVGRRTTree_Threshold > pdist([Qvis; robotTarget]))
            isDone = true;
            SVGRRTTree_Goals = [SVGRRTTree_Goals; SVGRRTTree(Qnew2_index)];
        end
    else
        % Add Qnew to tree
        Qnew_index = size(SVGRRTTree,2)+1;
        SVGRRTTree(Qnew_index).point = Qnew;
        SVGRRTTree(Qnew_index).distance = min_dist;
        SVGRRTTree(Qnew_index).cost = min_dist + SVGRRTTree(Qnear_index).cost;
        SVGRRTTree(Qnew_index).link = Qnear_index;
    end


    % Checks to see if we have reached the goal
    if (SVGRRTTree_Threshold > pdist([Qnew; robotTarget]))
        isDone = true;
        SVGRRTTree_Goals = [SVGRRTTree_Goals; SVGRRTTree(Qnew_index)];
    end
else 
    if (Qrand == robotTarget)
        % If we are looking to fast track via direct connection, but cannot
        % find a nearby point to connect, add 50 iterations and until we
        % find a valid connection 
        minIter = SVGRRTTree_Iterations + 50; 
    end
end

% Created based off: https://stackoverflow.com/questions/34474336/decide-if-a-point-is-on-or-close-enough-to-a-line-segment
function [on] = onEdge(xp, yp, x1, y1, x2, y2, thresh)
    line = [x2-x1 y2-y1];
    point = [xp-x1 yp-y1];
    len_sq = line(1)^2 + line(2)^2;
    dot_prod = point(1)*line(1) + point(2)*line(2);
    cross_prod = point(2)*line(1) - point(1)*line(2);
    dist = abs(cross_prod)/sqrt(len_sq);
    on = (dist <= thresh && dot_prod >= 0 && dot_prod <= len_sq);
end
