function [matrix] = transZ(z)

    matrix  = [[1 0 0 0]
               [0 1 0 0]
               [0 0 1 z]
               [0 0 0 1]];
end
