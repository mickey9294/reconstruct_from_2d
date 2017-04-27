function S = form_S( v1, v2 )
%FORM_S Summary of this function goes here
%   Detailed explanation goes here

x1 = v1(1);
y1 = v1(2);
z1 = v1(3);
x2 = v2(1);
y2 = v2(2);
z2 = v2(3);

S = zeros(3, 9);
S(1,:) = [0, 0, 0, -x2*z1, -y2*z1, -z1*z2, x2*y1, y1*y2, y1 * z2];
S(2,:) = [x2 * z1, y2*z1, z1*z2, 0, 0, 0, -x1*x2, -x1*y2, -x1*z2];
S(3,:) = [-x2*y1, -y1*y2, -y1*z2, x1*x2, x1*y2, x1*z2, 0, 0, 0];

end

