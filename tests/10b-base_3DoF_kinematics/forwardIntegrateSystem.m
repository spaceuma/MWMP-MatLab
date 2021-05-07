function x = forwardIntegrateSystem(x, u, dt)
    global wheelRadius dfx;

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
        x(13,i) = wheelRadius/2*(u(4,i-1) + u(5,i-1));
        x(14,i) = 0;
        x(15,i) = wheelRadius/(2*dfx)*(u(4,i-1) - u(5,i-1));
        % Arm Joints Position
        x(16:18,i) = x(16:18,i-1) + u(1:3,i-1)*dt;        
    end
end