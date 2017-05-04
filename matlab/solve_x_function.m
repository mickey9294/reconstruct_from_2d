function output = solve_x_function( x )
%SOLVE_X_FUNCTION Summary of this function goes here
%   Detailed explanation goes here

qiplus1 = load('qi_1.csv');
[Au, Bu, Cu] = form_unfix_constraints(x);

output = 0;
if size(Au, 1) > 0
    output = output + sumsqr(Au * qiplus1);
end
if size(Bu, 1) > 0
    result_B = Bu * qiplus1;
    output = output + sumsqr(result_B);
end
if size(Cu, 1) > 0
    output = output + sumsqr(Cu * qiplus1);
end

end

