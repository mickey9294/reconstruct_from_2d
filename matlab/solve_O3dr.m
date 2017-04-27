function solve_O3dr(Nf)
%SOLVE_EQUATIONS Summary of this function goes here
%   Detailed explanation goes here

x0 = ones(3 * Nf, 1);
x = lsqnonlin(@objective_function, x0);

dlmwrite('q0.csv', x);

end

