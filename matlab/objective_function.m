function output = objective_function( q )
%OBJECTIVE_FUNCTION Summary of this function goes here
%   Detailed explanation goes here

global Ad;
global Bd;
global Cd;
global E;
global G;

G_ = G + 1;

if size(Ad, 2) == size(q, 1)
    result_A = Ad * q;
else
    result_A = [];
end

if size(Bd, 2) == size(q, 1)
    result_B = Bd * q;
else
    result_B = [];
end

if size(Cd, 2) == size(q, 1)
    result_C = Cd * q;
else
    result_C = [];
end

if (size(E, 2)-1) == size(q, 1)
    result_E = E(:, 1:(end-1)) * q - E(:, end);
else
    result_E = [];
end

result_G = 0;
for i = 1:size(G_, 1)
    start_idx1 = 3 * (G_(i,1) - 1) + 1;
    start_idx2 = 3 * (G_(i,2) - 1) + 1;
    result_G  = result_G + dot(q(start_idx1: start_idx1 + 2), q(start_idx2: start_idx2+2));
end

result_N = zeros(size(q,1) / 3, 1);
for i = 1:size(result_N, 1)
    start = 3 * (i - 1) + 1;
    n = q(start:start + 2);
    result_N(i) = n(1)^2 + n(2)^2 + n(3)^2 - 1;
end

start = 1;
output = zeros(length(result_A)+length(result_B)+length(result_C)+length(result_E)+length(result_G)+length(result_N), 1);
output(start:size(result_A,1)) = result_A;
start = start + size(result_A,1);
output(start: start + size(result_B,1)-1) = result_B;
start = start + size(result_B,1);
output(start: start+size(result_C,1)-1) = result_C;
start = start + size(result_C,1);
output(start : start+size(result_E,1)-1) = result_E;
start = start + size(result_E, 1);
output(start:start+size(result_G,1)-1) = result_G;
start = start + size(result_G, 1);
output(start:start+size(result_N,1)-1) = result_N;

end

