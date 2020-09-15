function safe = isSafePath(x, y, mapResolution, dilatedObstMap)
   

    safe = 1;
    for i = 1:size(x,2)
        ix = round(x(i)/mapResolution)+1;
        iy = round(y(i)/mapResolution)+1;
        
        if dilatedObstMap(iy,ix) == 1
            safe = 0;
            break;
        end
    end
end

