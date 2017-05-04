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

global perspective_syms;
perspective_syms = load('perspective_syms.txt');

global face_circuits;
face_circuits = read_list('circuits.txt', size(q0,1)/3);
acc = @(x) x+1;
face_circuits = cellfun(acc, face_circuits, 'UniformOutput', false);

global precise_vertices;
precise_vertices = load('precise_id.csv');
precise_vertices = precise_vertices + 1;
global precise_sym_faces;
precise_sym_faces = load('precise_sym_faces.csv');

global Ad;
Ad = load('Ad.csv');
global Bd;
Bd = load('Bd.csv');
global Cd;
Cd = load('Cd.csv');
global E;
E = load('E.csv');
global G;
G = load('G.csv');

sigma = 1.0e-6;

global xi;
global qi;
% global xiplus1;
% global qiplus1;

xi = x0;
qi = q0;
i = 0;
while true
   qiplus1 = solve_q();
   xiplus1 = solve_x();
   
   Fiplus1 = F_q_x(qiplus1, xiplus1);
   Fi = F_q_x(qi, xi);
   delta = abs(Fiplus1 - Fi);
   if delta < sigma
       break;
   else
       qi = qiplus1;
       xi = xiplus1;
%        dlmwrite('xi.csv', xi);
%        dlmwrite('qi.csv', qi);
   end
   i = i + 1;
end

dlmwrite('xi.csv', xi);
dlmwrite('qi.csv', qi);

end

