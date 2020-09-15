function cost = getMeanTotalCost(x, i, mapResolution, totalCostMap)

    if size(x,2) > 1
        if i == 1
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            cAux1 = getTotalCost(xAux1, yAux1, mapResolution, totalCostMap);

            xAux4 = (x(1,i)+x(1,i+1))/2;
            yAux4 = (x(2,i)+x(2,i+1))/2;

            cAux4 = getTotalCost(xAux4, yAux4, mapResolution, totalCostMap);

            xAux5 = x(1,i+1);
            yAux5 = x(2,i+1);

            cAux5 = getTotalCost(xAux5, yAux5, mapResolution, totalCostMap);

            cost = (cAux1 +  cAux4 + cAux5)/3;
        elseif i == size(x,2)
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            cAux1 = getTotalCost(xAux1, yAux1, mapResolution, totalCostMap);

            xAux2 = (x(1,i)+x(1,i-1))/2;
            yAux2 = (x(2,i)+x(2,i-1))/2;

            cAux2 = getTotalCost(xAux2, yAux2, mapResolution, totalCostMap);

            xAux3 = x(1,i-1);
            yAux3 = x(2,i-1);

            cAux3 = getTotalCost(xAux3, yAux3, mapResolution, totalCostMap);

            cost = (cAux1 +  cAux2 + cAux3)/3;
        else
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            cAux1 = getTotalCost(xAux1, yAux1, mapResolution, totalCostMap);

            xAux2 = (x(1,i)+x(1,i-1))/2;
            yAux2 = (x(2,i)+x(2,i-1))/2;

            cAux2 = getTotalCost(xAux2, yAux2, mapResolution, totalCostMap);

            xAux3 = x(1,i-1);
            yAux3 = x(2,i-1);

            cAux3 = getTotalCost(xAux3, yAux3, mapResolution, totalCostMap);
            
            xAux4 = (x(1,i)+x(1,i+1))/2;
            yAux4 = (x(2,i)+x(2,i+1))/2;

            cAux4 = getTotalCost(xAux4, yAux4, mapResolution, totalCostMap);

            xAux5 = x(1,i+1);
            yAux5 = x(2,i+1);

            cAux5 = getTotalCost(xAux5, yAux5, mapResolution, totalCostMap);
            
            cost = (cAux1 +  cAux2 + cAux3 + cAux4 + cAux5)/5;
        end
    else
        cost = getTotalCost(x(1,i), x(2,i), mapResolution, totalCostMap);
    end
end