function [intersectionPose, intersectionIndex1, intersectionIndex2] = ...
                            getIntesection(path1,path2, waypointSeparation)
% GETINTERSECTION Get the crossing waypoint between two trajectories
% Given two different paths "path1" and "path2", and the distance
% separation between consecutive waypoints "waypointSeparation", the
% waypoint where the two paths intersect is computed
    found = false;
    for i = 1:size(path1,2)
        for j = 1:size(path2,2)
            if(norm(path1(1:2,i)-path2(1:2,j)) < waypointSeparation)
                intersectionPose = path1(1:2,i);
                intersectionIndex1 = i;
                intersectionIndex2 = j;
                found = true;
                break;
            end
%             if(i > 1 && j > 1)
%                 a = (path1(1:2,i-1) + path1(1:2,i))/2;
%                 b = (path2(1:2,j-1) + path2(1:2,j))/2;
%                 if(norm(a-b) < waypointSeparation)
%                     intersectionPose = path1(1:2,i);
%                     intersectionIndex1 = i;
%                     intersectionIndex2 = j;
%                     found = true;
%                     break;                
%                 end
%             end
        end
        if found
            break;
        end
    end
    if ~found
        warning('No intesection found between the trajectories.');
    end
end

