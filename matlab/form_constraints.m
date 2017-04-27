function [ Au, Bu, Cu ] = form_constraints( xi, edges, vert_face_map, face_parallel_groups, f )
%FORM_CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

Nf = length(face_parallel_groups);
% Form Au
num_faces_related = cellfun('length', vert_face_map);
Au = zeros(sum(num_faces_related - 1), 3 * Nf);
idx = 1;
for i = 1:length(vert_face_map)
    x_aug = [xi(i, :), -f]';
    for j = 1:num_faces_related(i)-1
        face_id_1 = vert_face_map{i}(j);
        face_id_1 = (face_id_1 - 1) * 3 + 1;
        face_id_2 = vert_face_map{i}(j + 1);
        face_id_2 = (face_id_2 - 1) * 3 + 1;
        Au(idx, face_id_1:(face_id_1 + 2)) = x_aug';
        Au(idx, face_id_2:(face_id_2 + 2)) = -x_aug';
        idx = idx + 1;
    end
end

% Form Bu
Bu = [];

%Form Cu
num_face_parallel_lines = cellfun('length', face_parallel_groups);
num_pairs = sum(num_face_parallel_lines / 2);
K = [f,0,0;0,f,0;0,0,1];
Cu = zeros(sum(num_pairs), 3 * Nf);
idx = 1;
for i = 1:length(face_parallel_groups)
    for j = 1:2:num_face_parallel_lines(i)
        line_1_id = face_parallel_groups{i}(j);
        line_1 = edges(line_1_id, :);
        p1 = xi(line_1(1),:);
        q1 = xi(line_1(2), :);
        l1 = line_equation(p1, q1);
        
        line_2_id = face_parallel_groups{i}(j + 1);
        line_2 = edges(line_2_id,:);
        p2 = xi(line_2(1), :);
        q2 = xi(line_2(2), :);
        l2 = line_equation(p2, q2);
        
        v = cross(l1, l2);
        v(1) = v(1) / v(3);
        v(2) = v(2) / v(3);
        v(3) = 1;
        R = K \ v;

        start = (i - 1) * 3 + 1;
        Cu(idx, start:(start + 2)) = R';
        idx = idx + 1;
    end
end

end

