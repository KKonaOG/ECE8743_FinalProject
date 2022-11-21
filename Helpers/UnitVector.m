function [u_hat] = UnitVector(x, y)
    magnitude = sqrt(x.^2 + y.^2);
    u_hat = [x / magnitude, y / magnitude];
end

