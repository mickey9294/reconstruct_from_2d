function [Aeq, beq] = form_fix_constraints( )
%FORM_FIX_CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

global E;
global Ad;
global Bd;
global Cd;

Aeq = [Ad; Bd; Cd; E(:, 1:(end-1))];
beq = zeros(size(Aeq,1),1);
beq(end) = E(:, end);

end

