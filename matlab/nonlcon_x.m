function [ c, ceq ] = nonlcon_x( x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global x0;

ceq = [];
xd = x - x0;
c = xd(:,1).^2 + xd(:,2) - 49;

end

