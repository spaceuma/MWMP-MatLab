function [switchPose, switchIndex] = getSwitch(originalPath,candidatePath)
    dist = 999;
    for i = 1:size(originalPath,2)
        if(norm(originalPath(1:2,i)-candidatePath(1:2,1)) < dist)
            switchPose = originalPath(1:2,i);
            switchIndex = i;
            dist = norm(originalPath(1:2,i)-candidatePath(1:2,1));
        end
    end

end

