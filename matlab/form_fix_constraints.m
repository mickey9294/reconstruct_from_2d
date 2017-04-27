function [Aeq, beq] = form_fix_constraints( Ad, Bd, Cd, E )
%FORM_FIX_CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

num_rows = size(Ad,1) + size(Bd,1) + size(Cd,1) + size(E,1);

E_real = E(:, 1:(end - 1));
beq = zeros(num_rows, 1);
beq(end - size(E,1) + 1 : end) = E(:, end);

Aeq = [Ad; Bd; Cd; E_real];

end

