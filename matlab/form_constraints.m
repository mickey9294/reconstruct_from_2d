function [ Au, Bu, Cu ] = form_constraints( xi )
%FORM_CONSTRIANTS Summary of this function goes here
%   Detailed explanation goes here

global edges;
global vert_face_map;
global face_parallel_groups;
global f;
global perspective_syms;
global face_circuits;

Nf = length(face_parallel_groups);

% Form Au
num_faces_related = cellfun('length', vert_face_map);
Au = zeros(sum(num_faces_related - 1), 3 * Nf);
idx = 1;
for i = 1:size(xi,1)
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
K = [-f,0,0;0,-f,0;0,0,1];
Bu = zeros(sum(perspective_syms > 0) * 2, 3 * Nf);
count = 1;
for i = 1:Nf
    sym_idx = perspective_syms(i);
    if sym_idx > 0
        circuit = face_circuits{i};
        
        N = length(circuit);
        Si = zeros(3 * N, 9);
        for k = 1:N
            x1_idx = mod(k, N) + 1;
            vert_idx_1 = circuit(x1_idx);
            x1 = [xi(vert_idx_1,:)'; 1.0];
            x2_idx = mod(sym_idx - k, N) + 1;
            vert_idx_2 = circuit(x2_idx);
            x2 = [xi(vert_idx_2,:)'; 1.0];
            
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
        start = 3 * (i - 1) + 1;
        Bi(:, start:(start + 2)) = Ci;
        
        gstart = (count - 1) * 2 + 1;
        Bu(gstart:(gstart+1),:) = Bi;
        count = count + 1;
    end
end

%Form Cu
num_face_parallel_lines = cellfun('length', face_parallel_groups);
num_pairs = sum(num_face_parallel_lines / 2);
Cu = zeros(num_pairs, 3 * Nf);
idx = 1;
for i = 1:length(face_parallel_groups)
    for j = 1:2:num_face_parallel_lines(i)
        line_1_id = face_parallel_groups{i}(j);
        line_2_id = face_parallel_groups{i}(j + 1);
        
        line_1 = edges(line_1_id, :);
        line_2 = edges(line_2_id,:);

        p1 = xi(line_1(1),:);
        q1 = xi(line_1(2),:);
        
        l1 = line_equation(p1, q1);

        p2 = xi(line_2(1),:);
        q2 = xi(line_2(2),:);
        
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

