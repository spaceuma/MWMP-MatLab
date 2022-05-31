function [z1, y, z2] = getZYZ(T)
%getRPY Obtain the z1, y and z2 angles given a certain
%       transformation matrix T.
    sy = sqrt(T(3,1)^2 + T(3,2)^2);
    cy = T(3,3);
    y = atan2(sy,cy);
    
    sz2 = T(3,2)/sy;
    cz2 = -T(3,1)/sy;
    z2 = atan2(sz2,cz2);
    
    sz1 = T(2,3)/sy;
    cz1 = T(1,3)/sy;
    z1 = atan2(sz1,cz1);
end
