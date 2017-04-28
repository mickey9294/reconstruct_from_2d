function [psx, psl, check] = perspective_symmetry(vertices, face_circuit)
%PERSPECTIVE_SYMMETRY detect perspective symmetry in a planar face
%   Input arguments:
%       vertices        2d vertices coordinates
%       face_circuit    vertices circuit sequence of the face
%
%   Output arguments:
%       psp     perspective symmetry point
%       psl     perspective symmetry axis
%       check   whether the face is of perspective symmetry

N = length(face_circuit);

min_ci = Inf;
min_Hi = zeros(3,3);

for i = 1:N
    if i == 2
        continue;
    end
    Si = zeros(3 * N, 9);
    for k = 1:N
        x1_idx = mod(k, N) + 1;
        x1 = [vertices(face_circuit(x1_idx), :)'; 1.0];
        x2_idx = mod(i - k, N) + 1;
        x2 = [vertices(face_circuit(x2_idx),:)'; 1.0];
        
        row_start = 3 * (k - 1) + 1;
        Si(row_start:(row_start + 2), :) = form_S(x1, x2);
    end
    
    [evectors, evalues] = eig(Si' * Si);
    evalues = diag(evalues);
    
    [~, min_idx] = min(evalues);
    hi = evectors(:, min_idx);
    Hi = reshape(hi, 3, 3)';
    
    ci = 0;
    for k = 1:N
        x1_idx = mod(k, N) + 1;
        x1 = [vertices(face_circuit(x1_idx), :)'; 1.0];
        x2_idx = mod(i - k, N) + 1;
        x2 = vertices(face_circuit(x2_idx),:)';
        
        sym_x1 = Hi * x1;
        sym_x1 = sym_x1 / sym_x1(3);
        sym_x1 = sym_x1(1:2);
        ci = ci + norm(sym_x1 - x2);
    end
    
    if ci < min_ci
        min_ci = ci;
        min_Hi = Hi;
        min_i = i;
    end
end

if min_ci > 0.01
    psx = zeros(3,1);
    psl = zeros(3,1);
    check = false;
    return;
else
    [hevec, heval] = eig(min_Hi);
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
    if abs(psx(3)) > 1.0e-8
        psx = psx / psx(3);
    else
        check = false;
        return;
    end
    if abs(psl_p1(3)) > 1.0e-8 && abs(psl_p2(3)) > 1.0e-8
        psl_p1 = psl_p1 / psl_p1(3);
        psl_p2 = psl_p2 / psl_p2(3);
        psl = line_equation(psl_p1, psl_p2);
        check = true;
    else
        check = false;
    end
end

end

