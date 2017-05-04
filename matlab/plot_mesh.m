function h = plot_mesh(vertex,face,options)

% plot_mesh - plot a 3D mesh.
%
%   plot_mesh(vertex,face, options);
%
%   'options' is a structure that may contains:
%       - 'normal' : a (nvertx x 3) array specifying the normals at each vertex.
%       - 'edge_color' : a float specifying the color of the edges.
%       - 'face_color' : a float specifying the color of the faces.
%       - 'face_vertex_color' : a color per vertex or face.
%       - 'vertex'
%
%   See also: mesh_previewer.
%
%   Copyright (c) 2004 Gabriel Peyr?


if nargin<2
    error('Not enough arguments.');
end
if nargin<3
    options.null = 0;
end


% can flip to accept data in correct ordering
if (size(vertex,1)==3 || size(vertex,1)==2) && size(vertex,2)~=3
    vertex = vertex';
end
if size(face,1)==3 && size(face,2)~=3
    face = face';
end

if size(face,2)~=3 || (size(vertex,2)~=3 && size(vertex,2)~=2)
    error('face or vertex does not have correct format.');
end


if ~isfield(options, 'normal')
    options.normal = [];
end
normal = options.normal;

if ~isfield(options, 'face_color')
    options.face_color = 0.7;
end
face_color = options.face_color;

if ~isfield(options, 'edge_color')
    options.edge_color = 1;
end
edge_color = options.edge_color;

if ~isfield(options, 'face_vertex_color')
    options.face_vertex_color = zeros(size(vertex,1),1);
end
face_vertex_color = options.face_vertex_color;


if isempty(face_vertex_color)
    h = patch('vertices',vertex,'faces',face,'facecolor',[face_color face_color face_color],'edgecolor',[edge_color edge_color edge_color]);
else
    nverts = size(vertex,1);
    % vertex_color = rand(nverts,1);
    if size(face_vertex_color,1)==size(vertex,1)
        shading_type = 'interp';
    else
        shading_type = 'flat';
    end
    if isfield(options,'noedge')&& options.noedge == 1
    h = patch('vertices',vertex,'faces',face,'FaceVertexCData',face_vertex_color, 'FaceColor','flat','EdgeColor','none');
    else
    h = patch('vertices',vertex,'faces',face,'FaceVertexCData',face_vertex_color, 'FaceColor','flat');    
    end
end
if isfield(options, 'keypoints')
    k = options.keypoints;
    hold on;
    for i = 1:size(k,1)
        plot3(k(i,1),k(i,2),k(i,3),'.g','MarkerSize',5);
    end
    hold off;
end
if isfield(options, 'keypoints2')
    k = options.keypoints2;
    hold on;
    for i = 1:size(k,1)
        plot3(k(i,1),k(i,2),k(i,3),'.b','MarkerSize',5);
    end
    hold off;
end

if isfield(options, 'sphere')
    k = options.sphere;
    for i = 1:size(k,1)
        [x,y,z]=sphere(30);
        X=x*k(i,4)+k(i,1);
        Y=y*k(i,4)+k(i,2);
        Z=z*k(i,4)+k(i,3);
        hold on;
        surf(X,Y,Z,'facecolor',[k(i,5),k(i,6),k(i,7)],'edgecolor','none','facealpha',0.3);
        hold off;
    end
end

if isfield(options, 'obb')
    obbs = options.obb;
    for i = 1:length(obbs.corner)
        vm = obbs.corner{i};
        fm = [1 2 3 4;3 4 5 6;5 6 7 8;7 8 1 2;1 4 5 8;2 3 6 7];
        for j = 1:8
            colorface(j,:) = obbs.color(i,:);
        end
        hold on;
        patch('vertices',vm,'faces',fm,'FaceVertexCData',colorface,'FaceColor','flat','EdgeColor','none','facealpha',0.3); 
        hold off;
    end
end


colormap gray(256);
lighting phong;
camlight infinite; 
camproj('perspective');
axis square; 
axis off;

if ~isempty(normal)
    % plot the normals
    hold on;
    quiver3(vertex(:,1),vertex(:,2),vertex(:,3),normal(:,1),normal(:,2),normal(:,3),0.8);
    hold off;
end

axis tight;
axis equal;
%cameramenu;
set(gcf,'color',[1,1,1]);