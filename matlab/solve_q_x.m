function solve_q_x()
%SOLVE_Q_X Summary of this function goes here
%   Detailed explanation goes here

global x0;
x0 = load('x0.csv');
q0 = load('q0.csv');

dlmwrite('xi.csv', x0);
dlmwrite('qi.csv', q0);

global vert_face_map;
vert_face_map = read_list('vert_face_map.txt', size(x0, 1));
acc = @(x) x+1;
vert_face_map = cellfun(acc, vert_face_map, 'UniformOutput', false);

global f;
f = load('f.csv');
global edges;
edges = load('edges.csv');
edges = edges + 1;

global face_parallel_groups;
face_parallel_groups = read_list('face_parallel_groups.txt', size(q0,1)/3);
face_parallel_groups = cellfun(acc, face_parallel_groups, 'UniformOutput', false);

global Ad;
Ad = load('A.csv');
global Bd;
Bd = load('B.csv');
global Cd;
Cd = load('C.csv');
global E;
E = load('E.csv');
global G;
G = load('G.csv');

sigma = 0.004;

% global xi;
% global qi;
% global xiplus1;
% global qiplus1;

xi = x0;
qi = q0;
i = 0;
while true
   qiplus1 = solve_q(qi);
   xiplus1 = solve_x(xi);
   
   delta = abs(F_q_x(qiplus1, xiplus1) - F_q_x(qi, xi));
   if delta < sigma
       break;
   else
       qi = qiplus1;
       xi = xiplus1;
       dlmwrite('xi.csv', xi);
       dlmwrite('qi.csv', qi);
   end
   i = i + 1;
end

end

