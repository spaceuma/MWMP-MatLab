function modulatedYaw = modulateYaw(yaw,reference)
    diff = reference - yaw;
    modulatedYaw = yaw;
    while diff > pi
        modulatedYaw = modulatedYaw + 2*pi;
        diff = reference - modulatedYaw;
    end
    while diff < -pi
        modulatedYaw = modulatedYaw - 2*pi;
        diff = reference - modulatedYaw;
    end
end

