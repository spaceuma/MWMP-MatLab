function [xCost, yCost] = getGradientTotalCost(x, y, mapResolution, gX, gY)


    ix = round(x/mapResolution)+1;
    iy = round(y/mapResolution)+1;
    
    if(ix>size(gX,2)-2)
        ix = size(gX,2)-2;
    end
    if(ix<3)
        ix = 3;
    end
    if(iy>size(gY,1)-2)
        iy = size(gY,1)-2;
    end
    if(iy<3)
        iy = 3;
    end
    
    xCost = gX(iy,ix);
    yCost = gY(iy,ix);
    
end