function cost = getCost(x, y)
    global costMap;
    mapResolution = 0.05;
        
    ix = round(x/mapResolution)+1;
    iy = round(y/mapResolution)+1;
    
    cost = costMap(ix,iy);
end