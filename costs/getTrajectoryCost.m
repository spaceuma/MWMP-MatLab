function cost = getTrajectoryCost(path, x, prevPose)

    headingCost = abs(prevPose(3)-path(3,1));
    distCost = DiscreteFrechetDist(path.', x.');
    cost = headingCost + distCost;
end

