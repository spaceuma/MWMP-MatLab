function [xCost, yCost] = getGradientMeanTotalCost(x, i, mapResolution, gX, gY)
    
    if size(x,2) > 1
        if i == 1
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            [xCAux1, yCAux1] = getGradientTotalCost(xAux1, yAux1, mapResolution, gX, gY);

            xAux4 = (x(1,i)+x(1,i+1))/2;
            yAux4 = (x(2,i)+x(2,i+1))/2;

            [xCAux4, yCAux4] = getGradientTotalCost(xAux4, yAux4, mapResolution, gX, gY);

            xAux5 = x(1,i+1);
            yAux5 = x(2,i+1);

            [xCAux5, yCAux5] = getGradientTotalCost(xAux5, yAux5, mapResolution, gX, gY);

            xCost = (xCAux1 +  xCAux4 + xCAux5)/3;
            yCost = (yCAux1 +  yCAux4 + yCAux5)/3;
        elseif i == size(x,2)
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            [xCAux1, yCAux1] = getGradientTotalCost(xAux1, yAux1, mapResolution, gX, gY);

            xAux2 = (x(1,i)+x(1,i-1))/2;
            yAux2 = (x(2,i)+x(2,i-1))/2;

            [xCAux2, yCAux2] = getGradientTotalCost(xAux2, yAux2, mapResolution, gX, gY);

            xAux3 = x(1,i-1);
            yAux3 = x(2,i-1);

            [xCAux3, yCAux3] = getGradientTotalCost(xAux3, yAux3, mapResolution, gX, gY);

            xCost = (xCAux1 + xCAux2 + xCAux3)/3;
            yCost = (yCAux1 + yCAux2 + yCAux3)/3;
        else
            xAux1 = x(1,i);
            yAux1 = x(2,i);

            [xCAux1, yCAux1] = getGradientTotalCost(xAux1, yAux1, mapResolution, gX, gY);

            xAux2 = (x(1,i)+x(1,i-1))/2;
            yAux2 = (x(2,i)+x(2,i-1))/2;

            [xCAux2, yCAux2] = getGradientTotalCost(xAux2, yAux2, mapResolution, gX, gY);

            xAux3 = x(1,i-1);
            yAux3 = x(2,i-1);

            [xCAux3, yCAux3] = getGradientTotalCost(xAux3, yAux3, mapResolution, gX, gY);

            xAux4 = (x(1,i)+x(1,i+1))/2;
            yAux4 = (x(2,i)+x(2,i+1))/2;

            [xCAux4, yCAux4] = getGradientTotalCost(xAux4, yAux4, mapResolution, gX, gY);

            xAux5 = x(1,i+1);
            yAux5 = x(2,i+1);

            [xCAux5, yCAux5] = getGradientTotalCost(xAux5, yAux5, mapResolution, gX, gY);

            xCost = (xCAux1 + xCAux2 + xCAux3 + xCAux4 + xCAux5)/5;
            yCost = (yCAux1 + yCAux2 + yCAux3 + yCAux4 + yCAux5)/5;
        end
    else
        [xCost, yCost] = getGradientTotalCost(x(1,i), x(2,i), mapResolution, gX, gY);
    end
end

