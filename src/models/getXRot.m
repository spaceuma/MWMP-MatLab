function Rx = getXRot(angle)
%getXRot Generates a X rotation matrix Rx(4x4) given a certain turn angle

    Rx = [1 0          0           0;
          0 cos(angle) -sin(angle) 0;
          0 sin(angle)  cos(angle) 0;          
          0 0          0           1];
      
    for i = 1:4  
        for j = 1:4
            if isnumeric(Rx(i,j))
                if(abs(Rx(i,j)) < 1e-6) 
                    Rx(i,j) = 0;
                end
            end
        end
    end
end