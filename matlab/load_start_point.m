function load_start_point( image_name )
%LOAD_START_POINT Summary of this function goes here
%   Detailed explanation goes here

start_path = fullfile('..\start_points', [image_name, '.csv']);
q0 = load(start_path);
dlmwrite('q0.csv', q0);

end

