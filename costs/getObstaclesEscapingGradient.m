function [gOMx, gOMy] = getObstaclesEscapingGradient(obstMap)
    notObstMap = 1-obstMap;
    distX = zeros(size(notObstMap));
    for i = 1:size(notObstMap,1)
        distX(i,:) = 1 + bwdist(notObstMap(i,:));
    end

    [gOMx, ~] = calculateGradientAxis(distX, 1);

    distY = zeros(size(notObstMap));
    for j = 1:size(notObstMap,2)
        distY(:,j) = 1 + bwdist(notObstMap(:,j));
    end

    [~, gOMy] = calculateGradientAxis(distY, 2);
%     gOMx = -gOMx;
%     gOMy = -gOMy;
end