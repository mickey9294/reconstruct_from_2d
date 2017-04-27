function output = solve_x_function( x )
%SOLVE_X_FUNCTION Summary of this function goes here
%   Detailed explanation goes here

global vert_face_map;
global f;
global edges;
global face_parallel_groups;

qiplus1 = load('qi_1.csv');
[Au, Bu, Cu] = form_constraints(x, edges, vert_face_map, face_parallel_groups, f);

output = 0;
if size(Au, 1) > 0
    output = output + sumsqr(Au * qiplus1);
end
if size(Bu, 1) > 0
    output = output + sumsqr(Bu * qiplus1);
end
if size(Cu, 1) > 0
    output = output + sumsqr(Cu * qiplus1);
end

end

