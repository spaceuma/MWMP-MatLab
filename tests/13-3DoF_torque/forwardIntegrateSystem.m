function x = forwardIntegrateSystem(x, u, dt, g)

    % Forward integrate system dynamics
    Jac = zeros(6,3,size(x,2));
    for i = 2:size(x,2)
        Jac(:,:,i-1) = jacobian3(x(7:9,i-1));
        % W2EE
        x(1:6,i) = x(1:6,i-1) + Jac(:,:,i-1)*x(10:12,i-1)*dt; 
        % Arm Joints Position
        x(7:9,i) = x(7:9,i-1) + dt*x(10:12,i-1);
        % Arm velocities
        x(10:12,i) = x(10:12,i-1) + dt*inv(getB3(x(7,i-1), x(8,i-1), x(9,i-1)))*...
                     (u(1:3,i-1) - ...
                     getC3(x(7,i-1), x(8,i-1), x(9,i-1),...
                           x(10,i-1), x(11,i-1), x(12,i-1))*x(10:12,i-1) - ...
                     getG3(x(7,i-1), x(8,i-1), x(9,i-1), g));
    end
end