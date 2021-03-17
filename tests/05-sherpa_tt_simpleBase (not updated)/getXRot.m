function Rx = getXRot(angle)
    Rx = [1 0          0           0;
          0 cos(angle) -sin(angle) 0;
          0 sin(angle)  cos(angle) 0;          
          0 0          0           1];
      
    for i = 1:4  
        for j = 1:4
            if(abs(Rx(i,j)) < 1e-6) 
                Rx(i,j) = 0;
            end
        end
    end
end