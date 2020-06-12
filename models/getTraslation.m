function T = getTraslation(pos)
    T = [1 0 0 pos(1);
         0 1 0 pos(2);
         0 0 1 pos(3);
         0 0 0 1];
end