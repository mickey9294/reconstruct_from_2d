function [ c, ceq ] = nonlcon_q( q )
%NONLCON_Q Summary of this function goes here
%   Detailed explanation goes here

c = [];

global E;

ceq = E(:, 1:(end - 1)) * q - E(:,end);
end

