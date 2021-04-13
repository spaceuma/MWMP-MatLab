function [T01, T02, T03, T04 ,T05] = directo5(q)
    % Modelo cinematico directo
    global a1;
    global a2;
    global a3;
    global a4;
    global a5;


    T01=[cos(q(1)) -sin(q(1)) 0 cos(q(1))*a1 ;
         sin(q(1)) cos(q(1)) 0 sin(q(1))*a1 ;
         0 0 1 0 ;
         0 0 0 1];

    T12=[cos(q(2)) -sin(q(2)) 0 cos(q(2))*a2 ;
         sin(q(2)) cos(q(2)) 0 sin(q(2))*a2 ;
         0 0 1 0 ;
         0 0 0 1];

    T23=[cos(q(3)) -sin(q(3)) 0 cos(q(3))*a3 ;
         sin(q(3)) cos(q(3)) 0 sin(q(3))*a3 ;
         0 0 1 0 ;
         0 0 0 1];
     
    T34=[cos(q(4)) -sin(q(4)) 0 cos(q(4))*a4 ;
         sin(q(4)) cos(q(4)) 0 sin(q(4))*a4 ;
         0 0 1 0 ;
         0 0 0 1];
     
    T45=[cos(q(5)) -sin(q(5)) 0 cos(q(5))*a5 ;
         sin(q(5)) cos(q(5)) 0 sin(q(5))*a5 ;
         0 0 1 0 ;
         0 0 0 1];

    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
end