% [verts_3d, faces] = read_off('..\shape.off');
% q = load('qi_1.csv');
% Q = reshape(q, 3, 9);
% 
% obj_center = (verts_3d(4,:) + verts_3d(6,:)) / 2;
% obj_center = obj_center';
% obj_center2 = (verts_3d(9,:) + verts_3d(12,:)) / 2;
% obj_center2 = obj_center2';
% 
% figure
% hold on
% grid on
% 
% xlabel('x');
% ylabel('y');
% zlabel('z');
% 
% camproj('perspective')
% 
% opt.face_vertex_color = [0.5,0.5,0.5];
% opt.noedge = 1;
% plot_mesh(verts_3d, faces, opt);
% 
% for i=1:3
%    % i = 5;
%     n = Q(:, i);
%     n_end = obj_center +  1000000* n;
%     plot3([obj_center(1), n_end(1)],[obj_center(2), n_end(2)],[obj_center(3), n_end(3)]);
% end
% 
% for i=4:6
%    % i = 5;
%     n = Q(:, i);
%     n_end = obj_center2 +  100000* n;
%     plot3([obj_center2(1), n_end(1)],[obj_center2(2), n_end(2)],[obj_center2(3), n_end(3)]);
% end
% 
% for i=7:9
%    % i = 5;
%     n = Q(:, i);
%     n_end = obj_center +  100000* n;
%     plot3([obj_center(1), n_end(1)],[obj_center(2), n_end(2)],[obj_center(3), n_end(3)]);
% end


% R = rotx(180);
% plotCamera('Location', [0,0,0], 'Orientation', R, 'Size', 60, 'Color', 'b', 'Label', '2', 'Opacity', 0);
% 
q = load('..\q_test.csv');
Q = reshape(q, 3, length(q)/3);
for i = 1: size(Q, 2)
    len = norm(Q(:,i));
    Q(:,i) = Q(:,i) / len;
end

qgt = load('..\q_gt.csv');
Qgt = reshape(qgt, 3, length(qgt) / 3);
for i = 1: size(Qgt, 2)
    len = norm(Qgt(:,i));
    Qgt(:,i) = Qgt(:,i)/ len;
end

figure
axis equal
hold on
for i = 1:9
    plot3([0 Q(1,i)], [0 Q(2,i)], [0 Q(3,i)]);
    plot3([0.2 Qgt(1,i)], [0.2 Qgt(2,i)], [0.2 Qgt(3,i)]);
end


