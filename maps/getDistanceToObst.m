function distance = getDistanceToObst(x, y, mapResolution)

    global distMap;

    ix = round(x/mapResolution)+1;
    iy = round(y/mapResolution)+1;

    if(ix < 1 || iy < 1 || ix > size(distMap,2) || iy > size(distMap,1))
        distance = 0;
    else
        distance = distMap(iy,ix);
    end    
end