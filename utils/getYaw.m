function yaw = getYaw(path, yaw0)
    yaw = zeros(size(path,1),1);
    
    yaw(1) = yaw0;
    for i = 2:size(path,1)
        yaw(i) = atan2(path(i,2)-path(i-1,2), path(i,1)-path(i-1,1));
    end
end

