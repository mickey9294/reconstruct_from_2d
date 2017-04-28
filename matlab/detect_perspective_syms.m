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

K = [-f,0,0;0,-f,0;0,0,1];

B = [];
for i = 1:Nf
    [psx, psl, check] = perspective_symmetry(vertices, face_circuits{i});
    if check
        ci1 = K \ psx;
        ci2 = cross(K \ psx, K' * psl);
        Ci = [ci1, ci2]';
        Bi = zeros(2, 3* Nf);
        
        start = 3 * (i - 1) + 1;
        Bi(:, start:(start + 2)) = Ci;
        B = [B; Bi];
    end
end

if exist('B.csv', 'file')
    delete('B.csv');
end
if size(B, 1) > 0
    dlmwrite('B.csv', B);
end

end

