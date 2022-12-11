function [didCollide] = CheckCollision(Qnear, Qnew, Obstacles, MapSize)
    didCollide = false;
    
    % Check for Raw Point Collisions
    for obsInd = 1:numel(Obstacles)
        obstacle = Obstacles(obsInd);
        insideObstacle = inpolygon(Qnew(1), Qnew(2), obstacle.Vertices(:, 1), obstacle.Vertices(:, 2));
        if (insideObstacle)
            didCollide = true;
            break;
        end
    end

    % Check for Edge Traversal Collisions
    traversalX = linspace(Qnear(1), Qnew(1), MapSize^2); % Resolution is related to map_size
    traversalY = linspace(Qnear(2), Qnew(2), MapSize^2); % Resolution is related to map_size

    for obsInd = 1:numel(Obstacles)
        obstacle = Obstacles(obsInd);
        insideObstacle = inpolygon(traversalX, traversalY, obstacle.Vertices(:, 1), obstacle.Vertices(:, 2));
        if (any(insideObstacle==1))
            didCollide = true;
            break;
        end
    end
    return;
end

