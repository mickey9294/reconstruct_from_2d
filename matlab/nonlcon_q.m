function [ c, ceq ] = nonlcon_q( q )
%NONLCON_Q Summary of this function goes here
%   Detailed explanation goes here

c = [];
Nf = length(q) / 3;
ceq = zeros(Nf, 1);
row = 1;
for i = 1:3:length(q)
    n = q(i:(i + 2));
    ceq(row) = n(1)^2 + n(2)^2 + n(3)^2 - 1;
    row = row + 1;
end

end

