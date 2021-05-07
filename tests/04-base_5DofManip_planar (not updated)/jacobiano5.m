function J = jacobiano5(q)
    % Calculo del jacobiano geometrico
    global a1;
    global a2;
    global a3;
    global a4;
    global a5;


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
      
    p5 = [a1*cos(q(1))+a2*cos(q(1)+q(2))+a3*cos(q(1)+q(2)...
          +q(3))+a4*cos(q(1)+q(2)+q(3)+q(4))+a5*cos(q(1)+q(2)+q(3)+q(4)+q(5));
          a1*sin(q(1))+a2*sin(q(1)+q(2))+a3*sin(q(1)+q(2)...
          +q(3))+a4*sin(q(1)+q(2)+q(3)+q(4))+a5*sin(q(1)+q(2)+q(3)+q(4)+q(5));
          0];  
      
    z = [0;
         0;
         1];

    Jp = [cross(z,(p5-p0)) cross(z,(p5-p1)) cross(z,(p5-p2)) cross(z,(p5-p3)) cross(z,(p5-p4))];
    Jo = [z z z z z];
    
    J = [Jp(1:2,:);
         Jo(3,:)];
end