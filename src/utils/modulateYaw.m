function modulatedYaw = modulateYaw(yaw,reference)
%MODULATEYAW Ensure the given yaw is continuous, avoiding jumps
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

