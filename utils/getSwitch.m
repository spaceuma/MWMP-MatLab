function [switchPose, switchIndex] = getSwitch(originalPath,candidatePath,index)
    dist = 999;
    check_size = ceil(size(originalPath,2)/10);
    if index > check_size
        for i = index - check_size:index
            if(norm(originalPath(1:2,i)-candidatePath(1:2,1)) < dist)
                switchPose = originalPath(1:2,i);
                switchIndex = i;
                dist = norm(originalPath(1:2,i)-candidatePath(1:2,1));
            end
        end
    else
        for i = 1:index
            if(norm(originalPath(1:2,i)-candidatePath(1:2,1)) < dist)
                switchPose = originalPath(1:2,i);
                switchIndex = i;
                dist = norm(originalPath(1:2,i)-candidatePath(1:2,1));
            end
        end
    end

end

