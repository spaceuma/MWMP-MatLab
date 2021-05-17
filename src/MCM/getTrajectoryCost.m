function cost = getTrajectoryCost(path, x, switchIndex)
% GETTRAJECTORYCOST Get the cost of a given trajectory
%   To obtain the cost of a given trajectory "path", the current state "x"
%   is checked and compared, from the index "switchIndex".
%   Different costs are used for comparison, currently only using the
%   heading (UNDER DEVELOPMENT).

    headingCost = 0;    
    for i = 1:size(path,2)
        headingCost = abs(path(3,i) - x(3,i + switchIndex - 1));
    end
    cost = headingCost;    
end

