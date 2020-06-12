function [ Gx, Gy ] = calculateMapGradient(costMap)
    Gx = costMap;
    Gy = costMap;
%     Gnx = costMap;
%     Gny = costMap;
    for i = 1:size(costMap,2)
        for j = 1:size(costMap,1)
            if j == 1
                Gy(1,i) = costMap(2,i)-costMap(1,i);
            else
                if j == size(costMap,1)
                    Gy(j,i) = costMap(j,i)-costMap(j-1,i);
                else
                    if (costMap(j+1,i) == Inf)
                        if (costMap(j-1,i) == Inf)
                            Gy(j,i) = 0;
                        else
                            Gy(j,i) = costMap(j,i)-costMap(j-1,i);
                        end
%                         Gy(j,i) = 0;
                    else
                        if (costMap(j-1,i) == Inf)
                            Gy(j,i) = costMap(j+1,i)-costMap(j,i);
%                             Gy(j,i) = 0;
                        else
                            Gy(j,i) = (costMap(j+1,i)-costMap(j-1,i))/2;
                        end
                    end
                end
            end
            if i == 1
                Gx(j,1) = costMap(j,2)-costMap(j,1);
            else
                if i == size(costMap,2)
                    Gx(j,i) = costMap(j,i)-costMap(j,i-1);
                else
                    if (costMap(j,i+1) == Inf)
                        if (costMap(j,i-1) == Inf)
                            Gx(j,i) = 0;
                        else
                            Gx(j,i) = costMap(j,i)-costMap(j,i-1);
                        end
%                         Gx(j,i) = 0;
                    else
                        if (costMap(j,i-1) == Inf)
                            Gx(j,i) = costMap(j,i+1)-costMap(j,i);
%                             Gx(j,i) = 0;
                        else
                            Gx(j,i) = (costMap(j,i+1)-costMap(j,i-1))/2;
                        end
                    end
                end
            end
%             Gnx(j,i) = Gx(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
%             Gny(j,i) = Gy(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
        end
    end
end