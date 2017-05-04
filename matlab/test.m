verts_3d = load('D:\Projects\reconstruct_from_2d\x_gt.csv');
faces = load('D:\Projects\reconstruct_from_2d\faces_gt.csv');
faces = faces + 1;
q = load('D:\Projects\reconstruct_from_2d\q_gt.csv');
Q = reshape(q, 3, 9);

obj_center = (verts_3d(1,:) + verts_3d(6,:)) / 2;
obj_center = obj_center';
obj_center2 = (verts_3d(9,:) + verts_3d(14,:)) / 2;
obj_center2 = obj_center2';

figure
hold on
grid on

xlabel('x');
ylabel('y');
zlabel('z');

camproj('perspective')

opt.face_vertex_color = [0.5,0.5,0.5];
opt.noedge = 1;
plot_mesh(verts_3d, faces, opt);

for i=1:3
   % i = 5;
    n = Q(:, i);
    n_end = obj_center +  100000* n;
    plot3([obj_center(1), n_end(1)],[obj_center(2), n_end(2)],[obj_center(3), n_end(3)]);
end

for i=4:6
   % i = 5;
    n = Q(:, i);
    n_end = obj_center2 +  100000* n;
    plot3([obj_center2(1), n_end(1)],[obj_center2(2), n_end(2)],[obj_center2(3), n_end(3)]);
end

for i=7:9
   % i = 5;
    n = Q(:, i);
    n_end = obj_center +  100000* n;
    plot3([obj_center(1), n_end(1)],[obj_center(2), n_end(2)],[obj_center(3), n_end(3)]);
end


R = rotx(180);
plotCamera('Location', [0,0,0], 'Orientation', R, 'Size', 60, 'Color', 'b', 'Label', '2', 'Opacity', 0);
