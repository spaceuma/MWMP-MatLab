function Rz = getZRot(angle)
    Rz = [cos(angle) -sin(angle) 0 0;
          sin(angle)  cos(angle) 0 0;
          0           0          1 0;
          0           0          0 1];
      
    for i = 1:4  
        for j = 1:4
            if isnumeric(Rz(i,j))
                if abs(Rz(i,j)) < 1e-6 
                    Rz(i,j) = 0;
                end
            end
        end
    end
end