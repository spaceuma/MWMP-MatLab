function index = getDistanceIndex(path, goal, distance)
    index = 1;
    for i = size(path,1):-1:1
        dist = norm(goal-path(i,1:2));
        if dist > distance
            index = i;
            break;
        end
    end
end

