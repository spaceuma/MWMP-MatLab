function [intersectionPose, intersectionIndex1, intersectionIndex2] = getIntesection(path1,path2, waypointSeparation)
    found = false;
    if(size(path1,2) == size(path2,2))
        for i = 1:size(path1,2)
            for j = 1:size(path2,2)
                if(norm(path1(1:2,i)-path2(1:2,j)) < waypointSeparation)
                    intersectionPose = path1(1:2,i);
                    intersectionIndex1 = i;
                    intersectionIndex2 = j;
                    found = true;
                    break;
                end
            end
            if found
                break;
            end
        end
    else
        error('Path sizes do not match')
    end
end

