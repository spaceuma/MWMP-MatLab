function b = getSimpleLogBarrierCost(x, threshold, t, greater)
    % This function returns the cost of a logarithmic barrier function like
    % b(g(x)) = -1/t*Log(-g(x)); where g(x) < 0 is a condition to be
    % fullfilled by x. This condition is x < threshold if "greater" is 0 or
    % x > threshold if "greater" is 1.
    
    if greater == 0
        g = x-threshold;
        if(x < threshold)
            b = -1./t*log10(-g);
        else
            b = -(2./t*log10(1e-5) + abs(1./t*log10(-g)));
        end
    else
        g = threshold-x;
        if(x > threshold)            
            b = -1./t*log10(-g);
        else
            b = -(2./t*log10(1e-5) + abs(1./t*log10(-g)));
        end
    end

end