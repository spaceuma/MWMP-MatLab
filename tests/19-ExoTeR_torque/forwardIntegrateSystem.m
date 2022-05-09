function x = forwardIntegrateSystem(x, u, dt)
    global wheelRadius wheelMass vehicleMass rollingResistance dfx dfy g;

    % Forward integrate system dynamics
    Jac = zeros(6,5,size(x,2));
    alphaR = zeros(size(x,2),1);
    for i = 2:size(x,2)
        Jac(:,:,i-1) = jacobian5(x(16:20,i-1));
        alphaR(i-1) = atan2(dfy,dfy/tan(x(35,i-1))+2*dfx)+0.000000000001;
        while alphaR(i-1) > pi/2
            alphaR(i-1) = alphaR(i-1) - pi;
        end
        while alphaR(i-1) < -pi/2
            alphaR(i-1) = alphaR(i-1) + pi;
        end
        
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(6,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(6,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(3,i-1) - Jac(1,:,i-1)*x(21:25,i-1)*dt;

        % B2EE
        x(4:6,i) = x(4:6,i-1) + Jac(1:3,:,i-1)*x(21:25,i-1)*dt; 
        x(7,i) = x(20,i-1); 
        x(8,i) = x(17,i-1) + x(18,i-1) + x(19,i-1); 
        x(9,i) = x(16,i-1); 

        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;

        % Bspeed
        x(13,i) = wheelRadius/2*(cos(x(35,i-1))*x(31,i-1) + cos(alphaR(i-1))*x(32,i-1));
        x(14,i) = 0;
        x(15,i) = x(13,i-1)*tan(x(35,i-1))/(dfy + dfx*tan(x(35,i-1)));

        % Arm Joints Position
        x(16:20,i) = x(16:20,i-1) + x(21:25,i-1)*dt;

        % Arm velocities
        x(21:25,i) = x(21:25,i-1) + dt * inv(getB5(x(16:20,i-1)))*...
                             (u(1:5,i-1) - ...
                             getC5(x(16:20,i-1),x(21:25,i-1)) * x(21:25,i-1) - ...
                             u(8,i-1)*getG5(x(16:20,i-1)));

        % Arm accelerations
        x(26:30,i) = inv(getB5(x(16:20,i-1)))*...
                     (u(1:5,i-1) - ...
                     getC5(x(16:20,i-1),x(21:25,i-1)) * x(21:25,i-1) - ...
                     u(8,i-1)*getG5(x(16:20,i-1)));

        % Wheels speeds
        x(31,i) = x(31,i-1) + dt*(u(6,i-1) - u(8,i-1)*rollingResistance*vehicleMass*wheelRadius/6)/...
                             (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/6*wheelRadius*wheelRadius);
        x(32,i) = (x(31,i-1) + dt*(u(6,i-1) - u(8,i-1)*rollingResistance*vehicleMass*wheelRadius/6)/...
                             (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/6*wheelRadius*wheelRadius))*sin(x(35,i-1))/sin(alphaR(i-1));

        % Wheels accelerations
        x(33,i) = (u(6,i-1) - u(8,i-1)*rollingResistance*vehicleMass*wheelRadius/6)/...
                     (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/6*wheelRadius*wheelRadius);
        x(34,i) = (u(6,i-1) - u(8,i-1)*rollingResistance*vehicleMass*wheelRadius/6)/...
                     (getWheelInertia(wheelMass,wheelRadius)+vehicleMass/6*wheelRadius*wheelRadius)*sin(x(35,i-1))/sin(alphaR(i-1));
        
        % Steering Joints Position
        x(35,i) = x(35,i-1) + u(7,i-1)*dt;

        % W2EE yaw
        x(36,i) = x(12,i-1) + x(7,i-1);
    end
end