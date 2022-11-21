function [didCollide] = CheckCollision(Qnear, Qnew, Obstacles)
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
    traversalX = linspace(Qnear(1), Qnew(1), 20); % Minimum res of four times the maximum step size
    traversalY = linspace(Qnear(2), Qnew(2), 20); % Minimum res of four times the maximum step size
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

