function [switchPose, switchIndex] = getSwitch(originalPath,candidatePath,index)
%GETSWITCH Compute the starting waypoint where to update the reference 
% trajectory
%   Given an original path and a candidate, the waypoint where to start
%   updating the original trajectory is computed, in the neighbourhood of
%   the index where the candidate path was extracted

    dist = 999;
    
    % Index threshold
    check_size = ceil(size(originalPath,2)/10);
    
    % If the wayp is far enough from the index 1
    if index > check_size
        % For each of the wayp inside the threshold
        for i = index - check_size:index
            % The closest waypoint is selected
            if(norm(originalPath(1:2,i)-candidatePath(1:2,1)) < dist)
                switchPose = originalPath(1:2,i);
                switchIndex = i;
                dist = norm(originalPath(1:2,i)-candidatePath(1:2,1));
            end
        end
    else
        % For each of the wayp inside the threshold
        for i = 1:index
            % The closest waypoint is selected
            if(norm(originalPath(1:2,i)-candidatePath(1:2,1)) < dist)
                switchPose = originalPath(1:2,i);
                switchIndex = i;
                dist = norm(originalPath(1:2,i)-candidatePath(1:2,1));
            end
        end
    end

end

