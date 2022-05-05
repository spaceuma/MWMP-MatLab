function x = forwardIntegrateSystem(x, u, dt)
    global g;
    % Forward integrate system dynamics
    for i = 2:size(x,2)
        % W2EE
        x(1:6,i) = x(1:6,i-1) + jacobian3(x(7:9,i-1))*u(1:3,i-1)*dt; 
        % Arm Joints Position
        x(7:9,i) = x(7:9,i-1) + u(1:3,i-1)*dt;
        % Arm velocities
        x(10:12,i) = u(1:3,i-1);
        % Arm accelerations
        x(13:15,i) = (u(1:3,i-1)-x(10:12,i-1))/dt;
        % Arm torques        
        x(16:18,i) = getB3(x(7,i-1), x(8,i-1), x(9,i-1))*x(13:15,i-1) +...
                     getC3(x(7,i-1), x(8,i-1), x(9,i-1), u(1,i-1), u(2,i-1), u(3,i-1))*u(1:3,i-1) +...
                     getG3(x(7,i-1), x(8,i-1), x(9,i-1),g);
    end
end