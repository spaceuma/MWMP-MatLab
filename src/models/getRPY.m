function [roll, pitch, yaw] = getRPY(T)
%getRPY Obtain the roll, pitch and yaw angles given a certain
%       transformation matrix T.
    spitch = T(1,3);
    cpitch = sqrt(T(1,1)^2 + T(1,2)^2);
    pitch = atan2(spitch,cpitch);
    
    syaw = -T(1,2)/cpitch;
    cyaw = T(1,1)/cpitch;
    yaw = atan2(syaw,cyaw);
    
    sroll = -T(2,3)/cpitch;
    croll = T(3,3)/cpitch;
    roll = atan2(sroll,croll);
end
