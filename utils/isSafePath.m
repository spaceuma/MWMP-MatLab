function safe = isSafePath(x, y, mapResolution, dilatedObstMap)
   

    safe = 1;
    for i = 1:size(x,2)
        ix = round(x(i)/mapResolution)+1;
        iy = round(y(i)/mapResolution)+1;
        
        if(ix>size(dilatedObstMap,1)-2)
            ix = size(dilatedObstMap,1)-2;
        end
        if(ix<3)
            ix = 3;
        end
        if(iy>size(dilatedObstMap,2)-2)
            iy = size(dilatedObstMap,2)-2;
        end
        if(iy<3)
            iy = 3;
        end
        
        if dilatedObstMap(iy,ix) == 1
            safe = 0;
            break;
        end
    end
end

