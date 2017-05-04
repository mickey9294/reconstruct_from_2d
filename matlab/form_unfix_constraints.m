function [ Au, Bu, Cu ] = form_unfix_constraints( xi )
%FORM_CONSTRAINTS Form constraints matrices from current 2d vertices
%   Required global variables:
%       xi:             the current 2d vertices positions
%       edges:          the edges indicating which two vertices are
%                       connected as an edge
%       vert_face_map:  indicats which faces the vertex are on
%       face_parallel_groups: contains couple of pairs representing lines
%                             pair that are parallel to the face
%       f:              focal length
%       perspective_syms: the i-th row contains a number m indicating that on
%       the i-th face, the vertex_2 is symmetric to vertex_m

global edges;
global vert_face_map;
global face_parallel_groups;
global f;
global perspective_syms;
global face_circuits;
global imprecise_vertices;
global precise_sym_faces;

Nf = length(face_parallel_groups);
% Form Au
num_faces_related = cellfun('length', vert_face_map);
Au = zeros(sum(num_faces_related(imprecise_vertices) - 1), 3 * Nf);
idx = 1;
for i = 1:size(xi,1)
    vert_id = imprecise_vertices(i);
    
    x_aug = [xi(i, :), -f]';
    for j = 1:num_faces_related(vert_id)-1
        face_id_1 = vert_face_map{vert_id}(j);
        face_id_1 = (face_id_1 - 1) * 3 + 1;
        face_id_2 = vert_face_map{vert_id}(j + 1);
        face_id_2 = (face_id_2 - 1) * 3 + 1;
        Au(idx, face_id_1:(face_id_1 + 2)) = x_aug';
        Au(idx, face_id_2:(face_id_2 + 2)) = -x_aug';
        idx = idx + 1;
    end
end

% Form Bu
K = [-f,0,0;0,-f,0;0,0,1];
global x0;
Bu = zeros((sum(perspective_syms > 0) - length(precise_sym_faces)) * 2, 3 * Nf);
imprecise_sym_faces = setdiff(1:Nf, precise_sym_faces);
count = 1;
for i = 1:length(imprecise_sym_faces)
    face_id = imprecise_sym_faces(i);
    
    sym_idx = perspective_syms(face_id);
    if sym_idx > 0
        circuit = face_circuits{face_id};
        
        N = length(circuit);
        Si = zeros(3 * N, 9);
        for k = 1:N
            x1_idx = mod(k, N) + 1;
            vert_idx_1 = circuit(x1_idx);
            find_in_imprecise1 = find(imprecise_vertices == vert_idx_1);
            if isempty(find_in_imprecise1)
                x1 = [x0(vert_idx_1, :)'; 1.0];
            else
                x1 = [xi(find_in_imprecise1,:)'; 1.0];
            end
            x2_idx = mod(sym_idx - k, N) + 1;
            vert_idx_2 = circuit(x2_idx);
            find_in_imprecise2 = find(imprecise_vertices == vert_idx_2);
            if isempty(find_in_imprecise2)
                x2 = [x0(vert_idx_2,:)'; 1.0];
            else
                x2 = [xi(find_in_imprecise2,:)'; 1.0];
            end
            
            row_start = 3 * (k - 1) + 1;
            Si(row_start:(row_start + 2),:) = form_S(x1, x2);
        end
        
        [evectors, evalues] = eig(Si' * Si);
        evalues = diag(evalues);
    
        [~, min_idx] = min(evalues);
        hi = evectors(:, min_idx);
        Hi = reshape(hi, 3, 3)';
        
        [hevec, heval] = eig(Hi);
        heval = real(heval);
        hevec = real(hevec);
        heval = diag(heval);
        if abs(heval(2)-heval(3))<abs(heval(2)-heval(1))&&abs(heval(2)-heval(3))<abs(heval(3)-heval(1))
            psx = hevec(:, 1);
            psl_p1 = hevec(:,2);
            psl_p2 = hevec(:,3);
        elseif abs(heval(1)-heval(3))<abs(heval(2)-heval(1))&&abs(heval(1)-heval(3))<abs(heval(3)-heval(2))
            psx = hevec(:,2);
            psl_p1 = hevec(:,1);
            psl_p2 = hevec(:,3);
        else
            psx = hevec(:,3);
            psl_p1 = hevec(:,1);
            psl_p2 = hevec(:,2);
        end
        psx = psx / psx(3);
        psl_p1 = psl_p1 / psl_p1(3);
        psl_p2 = psl_p2 / psl_p2(3);
        psl = line_equation(psl_p1, psl_p2);
        
        ci1 = K \ psx;
        ci2 = cross(K \ psx, K' * psl);
        Ci = [ci1, ci2]';
        Bi = zeros(2, 3* Nf);
        start = 3 * (face_id - 1) + 1;
        Bi(:, start:(start + 2)) = Ci;
        
        gstart = (count - 1) * 2 + 1;
        Bu(gstart:(gstart+1),:) = Bi;
        count = count + 1;
    end
end

%Form Cu
num_face_parallel_lines = cellfun('length', face_parallel_groups);
num_pairs = sum(num_face_parallel_lines / 2);
global Cd;
Cu = zeros(num_pairs - size(Cd, 1), 3 * Nf);
idx = 1;
for i = 1:length(face_parallel_groups)
    for j = 1:2:num_face_parallel_lines(i)
        line_1_id = face_parallel_groups{i}(j);
        line_2_id = face_parallel_groups{i}(j + 1);
        
        line_1 = edges(line_1_id, :);
        line_2 = edges(line_2_id,:);
        
        if ~any(ismember([line_1(1),line_1(2),line_2(1),line_2(2)], imprecise_vertices) ~= 0)
            continue;
        end
        
        find_in_imprecise1 = find(imprecise_vertices == line_1(1));
        find_in_imprecise2 = find(imprecise_vertices == line_1(2));
        if isempty(find_in_imprecise1)
            p1 = x0(line_1(1),:);
        else
            p1 = xi(find_in_imprecise1,:);
        end
        if isempty(find_in_imprecise2)
            q1 = x0(line_1(2),:);
        else
            q1 = xi(find_in_imprecise2,:);
        end
        
        l1 = line_equation(p1, q1);

        find_in_imprecise1 = find(imprecise_vertices == line_2(1));
        find_in_imprecise2 = find(imprecise_vertices == line_2(2));
        if isempty(find_in_imprecise1)
            p2 = x0(line_2(1),:);
        else
            p2 = xi(find_in_imprecise1,:);
        end
        if isempty(find_in_imprecise2)
            q2 = x0(line_2(2),:);
        else
            q2 = xi(find_in_imprecise2,:);
        end
        
        l2 = line_equation(p2, q2);
        
        v = cross(l1, l2);
        v = v / v(3);
        R = K \ v;

        start = (i - 1) * 3 + 1;
        Cu(idx, start:(start + 2)) = R';
        idx = idx + 1;
    end
end

end

