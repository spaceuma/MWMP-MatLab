function J = jacobiano4(q)
    % Calculo del jacobiano geometrico
    a1 = 0.20;
    a2 = 0.15;
    a3 = 0.10;
    a4 = 0.02;

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
    p4 = [a1*cos(q(1))+a2*cos(q(1)+q(2))+a3*cos(q(1)+q(2)+q(3))+a4*cos(q(1)+q(2)+q(3)+q(4));
          a1*sin(q(1))+a2*sin(q(1)+q(2))+a3*sin(q(1)+q(2)+q(3))+a4*sin(q(1)+q(2)+q(3)+q(4));
          0];      
      
    z = [0;
         0;
         1];

    Jp = [cross(z,(p4-p0)) cross(z,(p4-p1)) cross(z,(p4-p2)) cross(z,(p4-p3))];
    Jo = [z z z z];
    
    J = [Jp(1:2,:);
         Jo(3,:)];
end