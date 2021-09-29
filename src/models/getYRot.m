function Ry = getYRot(angle)
%getYRot Generates a Y rotation matrix Ry(4x4) given a certain turn angle

    Ry = [ cos(angle) 0 sin(angle) 0;
           0          1 0          0
          -sin(angle) 0 cos(angle) 0;          
           0          0 0          1];
       
    for i = 1:4  
        for j = 1:4
            if isnumeric(Ry(i,j))
                if(abs(Ry(i,j)) < 1e-6) 
                    Ry(i,j) = 0;
                end
            end
        end
    end
end