function qiplus1 = solve_q(qi)
%SOLVE_Q Summary of this function goes here
%   Detailed explanation goes here

% Ad = load('A.csv');
% Bd = load('B.csv');
% Cd = load('C.csv');
% E = load('E.csv');

[Aeq, beq] = form_fix_constraints();
qiplus1 = fmincon(@solve_q_function, qi, [], [], Aeq, beq);

dlmwrite('qi_1.csv', qiplus1);

end