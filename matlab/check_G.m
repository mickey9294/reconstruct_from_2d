function [result, R] = check_G( G, q )
%CHECK_G Summary of this function goes here
%   Detailed explanation goes here

G = G + 1;
result = 0;
R = [];

for i = 1:size(G, 1)
    start_idx1 = 3 * (G(i,1) - 1) + 1;
    start_idx2 = 3 * (G(i,2) - 1) + 1;
    n1 = q(start_idx1: start_idx1 + 2);
    n2 = q(start_idx2: start_idx2+2);
    n1 = n1 / norm(n1);
    n2 = n2 / norm(n2);
    mid = dot(n1, n2);
    result = result + mid;
    R(i) = mid;
end

end

