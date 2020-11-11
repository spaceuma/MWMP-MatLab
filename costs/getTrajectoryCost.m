function cost = getTrajectoryCost(path, x)
    headingCost = 0;
    distCost = 0;
    
    n = size(path,2);
    
    headingCost = headingCost + abs(path(3,1) - x(3,1));
    for i = 2:n
        headingCost = headingCost + abs(path(3,i) - x(3,i));
%         distCost = distCost + norm(path(1:2,i) - path(1:2,i-1));
    end
    cost = headingCost/(n*pi) + distCost/(n);
end

