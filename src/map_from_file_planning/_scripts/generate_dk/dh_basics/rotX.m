function [matrix] = rotX(alfa)

    matrix  = [[1     0         0      0]
               [0 cos(alfa) -sin(alfa) 0]
               [0 sin(alfa)  cos(alfa) 0]
               [0     0         0      1]];

end
