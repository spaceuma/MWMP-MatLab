function cost = getTrajectoryCost(path, x)
    headingCost = 0;
    smoothCost = 0;
    distCost = 0;
    
    n = size(path,2);
    
    headingCost = headingCost + abs(path(3,1) - x(3,1));

    for i = 2:n
        smoothCost = smoothCost + abs(path(3,i) - path(3,i-1));
%         distCost = distCost + norm(path(1:2,i) - x(1:2,i));
    end
    cost = headingCost/(pi) + smoothCost/(n*pi) + distCost/(0.3*n);
end

