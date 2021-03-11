function length = getLength(trajectory)
%GETLENGTH Compute the total length of a given trajectory
%   Euclidean distance is used to compute the total length of the input
%   trajectory, as the sum of the distances between the waypoints. The
%   input trajectory must be a 2D array with waypoints in each column, and
%   each waypoint should contain at least a first row with X positions and
%   a second one with Y positions

    n = size(trajectory,2);
    m = size(trajectory,1);
    
    if m < 2
        error(['Each waypoint must contain at least two rows:',...
              ' X position and Y position.']);
    end
    
    if n > 1
        length = 0;
        for i = 2:n
            dist = norm(trajectory(1:2,i)-trajectory(1:2,i-1));
            length = length + dist;
        end
    else
        error('The trajectory must contain at least 2 waypoints');
    end
end

