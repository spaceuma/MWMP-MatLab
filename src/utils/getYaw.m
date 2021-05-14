function yaw = getYaw(path, yaw0)
%GETYAW Computes the yaw of the given path, optionally setting the first
%wayp yaw
    switch nargin
        case 2
            yaw = zeros(size(path,1),1);
            yaw(1) = yaw0;
            for i = 2:size(path,1)
                yaw(i) = atan2(path(i,2)-path(i-1,2), path(i,1)-path(i-1,1));
            end
        case 1
            yaw = zeros(size(path,1),1);
            yaw(1) = atan2(path(2,2)-path(1,2), path(2,1)-path(1,1));
            for i = 2:size(path,1)
                yaw(i) = atan2(path(i,2)-path(i-1,2), path(i,1)-path(i-1,1));
            end
        otherwise
           error('Not enough input arguments.'); 
    end
end

