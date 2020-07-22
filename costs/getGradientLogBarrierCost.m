function bx = getGradientLogBarrierCost(x, threshold, t, greater)
    % This function returns the cost of a logarithmic barrier function like
    % b(g(x)) = -1/t*Log(-g(x)); where g(x) < 0 is a condition to be
    % fullfilled by x. This condition is x < threshold if "greater" is 0 or
    % x > threshold if "greater" is 1.
    
    if greater == 0
        g = x-threshold;
        if(x < threshold)
            bx = 1./(t*(-g)*log(10));
        else
            bx = 1./(t*(-g)*log(10));
%             bx = 1./(t*(1e-3)*log(10));
        end
    else
        g = threshold-x;
        if(x > threshold)
            bx = -1./(t*(-g)*log(10));
        else
            bx = -1./(t*(-g)*log(10));
%             bx = -1./(t*(1e-3)*log(10));
        end
    end

end