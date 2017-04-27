function output = solve_q_function( q )
%SOLVE_Q_FUNCTION Summary of this function goes here
%   Detailed explanation goes here

xi = load('xi.csv');
global vert_face_map;
global f;
global edges;
global G;
global face_parallel_groups;

[Au, Bu, Cu]= form_constraints(xi, edges, vert_face_map, face_parallel_groups, f);

output = 0;
if size(Au, 1) > 0
    output = output + sumsqr(Au * q);
end
if size(Bu,1) > 0
    output = output + sumsqr(Bu* q);
end

if size(Cu, 1) > 0
    output = output + sumsqr(Cu * q);
end

gsum = 0;
for i = 1:size(G,1)
    face_id_1 = G(i, 1);
    face_id_2 = G(i, 2);
    
    start1 = face_id_1 * 3 + 1;
    start2 = face_id_2 * 3 + 1;
    
    n1 = q(start1:(start1 + 2));
    n2 = q(start2:(start2 + 2));
    
    gsum = gsum + dot(n1, n2)^2;
end
output = output + gsum;

end

