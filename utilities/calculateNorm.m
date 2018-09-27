% Calculate the unit normal vector to the line joining two points.
%
% Maths from https://stackoverflow.com/questions/1243614/how-do-i-calculate-the-normal-vector-of-a-line-segment
function norm1 = calculateNorm(point1, point2)
    dx = point1(1) - point2(1);
    dy = point1(2) - point2(2);
    
    norm1 = [-dy dx]';
    
    % Normalise the vector.
    norm1 = norm1 / norm(norm1);
    
end