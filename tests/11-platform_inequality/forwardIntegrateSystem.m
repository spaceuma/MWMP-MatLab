function x = forwardIntegrateSystem(x, u, dt)
    global wheelRadius wheelMass vehicleMass rollingResistance dfx g;

    % Forward integrate system dynamics
    Jac = zeros(6,3,size(x,2));
    for i = 2:size(x,2)
        Jac(:,:,i-1) = jacobian3(x(16:18,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(3,i-1) + Jac(3,:,i-1)*u(1:3,i-1)*dt;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:3,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = wheelRadius/2*(cos(x(40,i-1))*u(4,i-1) + cos(x(42,i-1))*u(5,i-1));
        x(14,i) = - wheelRadius/2*(sin(x(40,i-1))*u(4,i-1) + sin(x(42,i-1))*u(5,i-1));
        x(15,i) = wheelRadius/(2*dfx)*(cos(x(40,i-1))*u(4,i-1) - cos(x(42,i-1))*u(5,i-1));
        % Arm Joints Position
        x(16:18,i) = x(16:18,i-1) + u(1:3,i-1)*dt;
        % Arm velocities
        x(19:21,i) = u(1:3,i-1);
        % Arm accelerations
        x(22:24,i) = (u(1:3,i-1)-x(19:21,i-1))/dt;
        % Arm torques        
        x(25:27,i) = getB3(x(16,i-1), x(17,i-1), x(18,i-1))*x(22:24,i-1) +...
                     getC3(x(16,i-1), x(17,i-1), x(18,i-1), u(1,i-1), u(2,i-1), u(3,i-1))*u(1:3,i-1) +...
                     getG3(x(16,i-1), x(17,i-1), x(18,i-1));
        % Wheels speeds
        x(28:29,i) = u(4,i-1);
        x(30:31,i) = u(5,i-1);
        % Wheels accelerations
        x(32,i) = (u(4,i-1) - x(28,i-1))/dt;
        x(33,i) = (u(4,i-1) - x(29,i-1))/dt;
        x(34,i) = (u(5,i-1) - x(30,i-1))/dt;
        x(35,i) = (u(5,i-1) - x(31,i-1))/dt;
        % Wheels torques
        x(36,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(32,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
        x(37,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(33,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
        x(38,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(34,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
        x(39,i) = (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius)*x(35,i-1)...
            + rollingResistance*vehicleMass*g*wheelRadius/4;
        % Steering Joints Position
        x(40:41,i) = x(40:41,i-1) + u(6,i-1)*dt;
        x(42:43,i) = x(42:43,i-1) + u(7,i-1)*dt;
    end
end