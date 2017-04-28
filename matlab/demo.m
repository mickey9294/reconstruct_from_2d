x = load('x0.csv');
edges = load('edges.csv');
edges = edges + 1;

nedges = size(edges,1);
lines = zeros(3, nedges);

for i=1: nedges
    v0_idx = edges(i, 1);
    v1_idx = edges(i, 2);
    v0 = x(v0_idx, :);
    v1 = x(v1_idx, :);
    lines(:, i) = line_equation(v0, v1);
end

parallel_groups = [1,3,11,9;5,7,8,6;4,12,10,2];

vp_list = java.util.ArrayList;

for i = 1:3
    vp_count = 0;
    vanishing_point = zeros(3, 1);
    for j = 1:4
        line_id_1 = parallel_groups(i, j);
        line_1 = lines(:, line_id_1);
        
        for k = j + 1 : 4
            line_id_2 = parallel_groups(i, k);
            line_2 = lines(:, line_id_2);
            
            vp = cross(line_1, line_2);
            vp = vp / vp(3);
            
            vanishing_point = vanishing_point + vp;
            vp_count = vp_count + 1;
        end
    end
    
    vanishing_point = vanishing_point / vp_count;
    vp_list.add(vanishing_point);
end

count = 0;
f = 0;
for i = 1:vp_list.size()
    vp1 = vp_list.get(i - 1);
    for j = (i + 1):vp_list.size()
        vp2 = vp_list.get(j - 1);
        
        mid = -dot(vp1, vp2);
        if mid >= 0
            f = f + sqrt(mid);
            count = count + 1;
        end
    end
end
if count > 0
    f = f / count;
end