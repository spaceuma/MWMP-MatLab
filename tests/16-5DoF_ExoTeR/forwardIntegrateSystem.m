function x = forwardIntegrateSystem(x, u, dt)
    global wheelRadius wheelMass vehicleMass rollingResistance dfx g;

    % Forward integrate system dynamics
    Jac = zeros(6,5,size(x,2));
    for i = 2:size(x,2)
        Jac(:,:,i-1) = jacobian5(x(16:20,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(3,i-1) + Jac(3,:,i-1)*u(1:5,i-1)*dt;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:5,i-1)*dt; 
        % W2C
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = wheelRadius/2*(u(6,i-1) + u(7,i-1));
        x(14,i) = 0;
        x(15,i) = wheelRadius/(2*dfx)*(u(6,i-1) - u(7,i-1));
        % Arm Joints Position
        x(16:20,i) = x(16:20,i-1) + u(1:5,i-1)*dt;
        % Arm velocities
        x(21:25,i) = u(1:5,i-1);
        % Arm accelerations
        x(26:30,i) = (u(1:5,i-1)-x(21:25,i-1))/dt;
        % Arm torques        
        x(31:35,i) = getB5(x(16:20,i-1))*x(26:30,i-1) +...
                     getC5(x(16:20,i-1), u(1:5,i-1))*u(1:5,i-1);
%                      getG5(x(16:20,i-1));
        % Wheels speeds
        x(36,i) = u(6,i-1);
        x(37,i) = u(7,i-1);
        % Wheels accelerations
        x(38,i) = (u(6,i-1) - x(36,i-1))/dt;
        x(39,i) = (u(7,i-1) - x(37,i-1))/dt;
        % Wheels torques
        x(40,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(38,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
        x(41,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(39,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
    end
end