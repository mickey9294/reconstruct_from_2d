function generate_apm_model( qi, Au, Bu, Cu, G, Ad, Bd, Cd, E )
%GENERATE_APM_MODEL Summary of this function goes here
%   Detailed explanation goes here
fileId = fopen('model.amp', 'w');
fprintf(fileId, 'Model\n');
fprintf(fileId, '  Variables\n');
for i = 1:length(qi)
    fprintf(fileId, '    q[%d] = %f, >=-1.0, <=1.0\n', i, qi(i));
end
fprintf(fileId, '  End Variables\n\n');


fclose(fileId);
end

