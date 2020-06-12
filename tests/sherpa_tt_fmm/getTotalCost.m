function cost = getTotalCost(x, y, mapResolution, totalCostMap)

    ix = round(x/mapResolution)+1;
    iy = round(y/mapResolution)+1;

    if(ix>size(totalCostMap,1)-2)
        ix = size(totalCostMap,1)-2;
    end
    if(ix<3)
        ix = 3;
    end
    if(iy>size(totalCostMap,2)-2)
        iy = size(totalCostMap,2)-2;
    end
    if(iy<3)
        iy = 3;
    end
    
    cost = totalCostMap(ix,iy);
end