function [ Gnx, Gny ] = calculateGradientAxis(map, axis)

    if axis == 0
        Gx = map;
        Gy = map;
        Gnx = map;
        Gny = map;
        for i = 1:size(map,2)
            for j = 1:size(map,1)
                if j == 1
                    Gy(1,i) = map(2,i)-map(1,i);
                else
                    if j == size(map,1)
                        Gy(j,i) = map(j,i)-map(j-1,i);
                    else
                        if (map(j+1,i) == Inf)
                            if (map(j-1,i) == Inf)
                                Gy(j,i) = 0;
                            else
                                Gy(j,i) = map(j,i)-map(j-1,i);
                            end
    %                         Gy(j,i) = 0;
                        else
                            if (map(j-1,i) == Inf)
                                Gy(j,i) = map(j+1,i)-map(j,i);
    %                             Gy(j,i) = 0;
                            else
                                Gy(j,i) = (map(j+1,i)-map(j-1,i))/2;
                            end
                        end
                    end
                end
                if i == 1
                    Gx(j,1) = map(j,2)-map(j,1);
                else
                    if i == size(map,2)
                        Gx(j,i) = map(j,i)-map(j,i-1);
                    else
                        if (map(j,i+1) == Inf)
                            if (map(j,i-1) == Inf)
                                Gx(j,i) = 0;
                            else
                                Gx(j,i) = map(j,i)-map(j,i-1);
                            end
    %                         Gx(j,i) = 0;
                        else
                            if (map(j,i-1) == Inf)
                                Gx(j,i) = map(j,i+1)-map(j,i);
    %                             Gx(j,i) = 0;
                            else
                                Gx(j,i) = (map(j,i+1)-map(j,i-1))/2;
                            end
                        end
                    end
                end
                Gnx(j,i) = Gx(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
                Gny(j,i) = Gy(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
            end
        end
    elseif axis == 1
        Gx = map;
        Gnx = map;
        Gny = map;

        for i = 1:size(map,2)
            for j = 1:size(map,1)                
                if i == 1
                    Gx(j,1) = map(j,2)-map(j,1);
                else
                    if i == size(map,2)
                        Gx(j,i) = map(j,i)-map(j,i-1);
                    else
                        if (map(j,i+1) == Inf)
                            if (map(j,i-1) == Inf)
                                Gx(j,i) = 0;
                            else
                                Gx(j,i) = map(j,i)-map(j,i-1);
                            end
    %                         Gx(j,i) = 0;
                        else
                            if (map(j,i-1) == Inf)
                                Gx(j,i) = map(j,i+1)-map(j,i);
    %                             Gx(j,i) = 0;
                            else
                                Gx(j,i) = (map(j,i+1)-map(j,i-1))/2;
                            end
                        end
                    end
                end
                Gnx(j,i) = Gx(j,i);
                Gny(j,i) = 0;
            end
        end
    elseif axis == 2
        Gy = map;
        Gnx = map;
        Gny = map;
        
        for i = 1:size(map,2)
            for j = 1:size(map,1)
                if j == 1
                    Gy(1,i) = map(2,i)-map(1,i);
                else
                    if j == size(map,1)
                        Gy(j,i) = map(j,i)-map(j-1,i);
                    else
                        if (map(j+1,i) == Inf)
                            if (map(j-1,i) == Inf)
                                Gy(j,i) = 0;
                            else
                                Gy(j,i) = map(j,i)-map(j-1,i);
                            end
    %                         Gy(j,i) = 0;
                        else
                            if (map(j-1,i) == Inf)
                                Gy(j,i) = map(j+1,i)-map(j,i);
    %                             Gy(j,i) = 0;
                            else
                                Gy(j,i) = (map(j+1,i)-map(j-1,i))/2;
                            end
                        end
                    end
                end                
                Gnx(j,i) = 0;
                Gny(j,i) = Gy(j,i);
            end
        end
    end
end