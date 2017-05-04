function xiplus1 = solve_x()
%SOLVE_X Summary of this function goes here
%   Detailed explanation goes here

global xi;
xiplus1 = fmincon(@solve_x_function, xi, [],[],[],[],[],[],@nonlcon_x);
%dlmwrite('xi_1.csv', xiplus1);

end

