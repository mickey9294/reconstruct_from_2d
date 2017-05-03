function detect_perspective_syms( Nf )
%DETECT_PERSPECTIVE_SYMS Detect all faces of perspective symmetry in an
%image
%   Input arguments:
%       Nf - the number of faces in total
%       face_circuits - vertices circuit sequence of each planar face(read
%                       from file circuits.csv)
%       vertices - 2D vertices on the image
%       f - the focal length
%
%   Output arguments:
%       B - perspective constraint matrix (write to file B.csv);

vertices = load('x0.csv');
face_circuits = read_list('circuits.txt', Nf);
acc = @(x) x+1;
face_circuits = cellfun(acc, face_circuits, 'UniformOutput', false);
f = load('f.csv');
precise_vertices = load('precise_id.csv');
precise_vertices = precise_vertices + 1;

K = [-f,0,0;0,-f,0;0,0,1];

B = [];
Bd = [];
sym_points = zeros(Nf, 1);
precise_sym_faces = [];
for i = 1:Nf
    [psx, psl, sym_point, check] = perspective_symmetry(vertices, face_circuits{i});
    sym_points(i) = sym_point;
    if check
        ci1 = K \ psx;
        ci2 = cross(K \ psx, K' * psl);
        Ci = [ci1, ci2]';
        Bi = zeros(2, 3* Nf);
        
        start = 3 * (i - 1) + 1;
        Bi(:, start:(start + 2)) = Ci;
        B = [B; Bi];
        
        if length(intersect(face_circuits{i}, precise_vertices)) == length(face_circuits{i})
            Bd = [Bd; Bi];
            precise_sym_faces = [precise_sym_faces; i];
        end
%         
%         S_path = ['evec_' num2str(i) '.csv'];
%         dlmwrite(S_path, min_Si);
%         H_path = ['eval_' num2str(i) '.csv'];
%         dlmwrite(H_path, min_Hi);
    end
end

if exist('B.csv', 'file')
    delete('B.csv');
end
if size(B, 1) > 0
    dlmwrite('B.csv', B);
end
if exist('Bd.csv', 'file')
    delete('Bd.csv');
end
if size(Bd, 1) > 0
    dlmwrite('Bd.csv', Bd);
end

if exist('perspective_syms.txt', 'file')
    delete('perspective_syms.txt');
end
dlmwrite('perspective_syms.txt', sym_points);

if exist('precise_sym_faces.csv', 'file')
    delete('precise_sym_faces.csv');
end
dlmwrite('precise_sym_faces.csv', precise_sym_faces);

end

