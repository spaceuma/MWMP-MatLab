function Ry = getYRot(angle)
    Ry = [ cos(angle) 0 sin(angle) 0;
           0          1 0          0
          -sin(angle) 0 cos(angle) 0;          
           0          0 0          1];
       
    for i = 1:4  
        for j = 1:4
            if(abs(Ry(i,j)) < 1e-6) 
                Ry(i,j) = 0;
            end
        end
    end
end