function J = jacobiano3(q)
    % Calculo del jacobiano geometrico
    a1 = 0.20;
    a2 = 0.15;
    a3 = 0.10;

    p0 = [0;
          0;
          0];

    p1 = [a1*cos(q(1));
          a1*sin(q(1));
          0];

    p2 = [a1*cos(q(1))+a2*cos(q(1)+q(2));
          a1*sin(q(1))+a2*sin(q(1)+q(2));
          0];
    p3 = [a1*cos(q(1))+a2*cos(q(1)+q(2))+a3*cos(q(1)+q(2)+q(3));
          a1*sin(q(1))+a2*sin(q(1)+q(2))+a3*sin(q(1)+q(2)+q(3));
          0];
    z = [0;
         0;
         1];

    Jp = [cross(z,(p3-p0)) cross(z,(p3-p1)) cross(z,(p3-p2))];
    Jo = [z z z];
    
    J = [Jp(1:2,:);
         Jo(3,:)];
end