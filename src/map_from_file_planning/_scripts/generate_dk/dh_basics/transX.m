function [matrix] = transX(x)

    matrix  = [[1 0 0 x]
               [0 1 0 0]
               [0 0 1 0]
               [0 0 0 1]];
end
