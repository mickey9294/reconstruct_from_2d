function [ line ] = line_equation( p, q )
%LINE_EQUATION get line equation of a 2D line
%   Detailed explanation goes here
if abs(p(1) - q(1)) < 1.0e-8
    if abs(p(1)) > 1.0e-8
        line = [-1.0/p(1); 0; 1.0];
    else
        line = [1.0; 0; -p(1)];
    end
else
    x1 = p(1);
    y1 = p(2);
    x2 = q(1);
    y2 = q(2);
    
    line = [(y2 - y1)/(x2 - x1); -1; -x1*(y2 - y1)/(x2 - x1) + y1];
    if abs(line(3)) > 1.0e-8
        line = line / line(3);
    end
end
end

