function dilatedMap = dilateObstMap(obstMap,safetyDistance,mapResolution)
se = strel('disk',round(safetyDistance/mapResolution));        
dilatedMap = imdilate(obstMap,se);
end