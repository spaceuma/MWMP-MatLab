function cost = getTrajectoryCost(path, x, switchIndex)

    headingCost = 0;
    distCost = 0;
%     distCost = DiscreteFrechetDist(path.', x.');
    
    for i = 1:size(path,2)
        headingCost = abs(path(3,i) - x(3,i + switchIndex - 1));
    end
    cost = distCost + headingCost;    
end

