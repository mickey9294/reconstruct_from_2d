function solve_O3dr(Nf)
%SOLVE_EQUATIONS Summary of this function goes here
%   Detailed explanation goes here

global Ad;
global Bd;
global Cd;
global E;
global G;
Ad = load('A.csv');
Bd = load('B.csv');
Cd = load('C.csv');
E = load('E.csv');
G = load('G.csv');

x0 = -1 + (1 + 1) * rand(3 * Nf, 1);
x = lsqnonlin(@objective_function, x0);

dlmwrite('q0.csv', x);

end

