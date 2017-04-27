function list = read_list( path, num_items )
%READ_LIST Summary of this function goes here
%   Detailed explanation goes here

fid = fopen(path);
textLine = fgets(fid);
lineCounter = 1;

if nargin >= 2
    list = cell(num_items, 1);
else
    list = {};
end

while ischar(textLine)
    numbers = sscanf(textLine, '%f');
    list{lineCounter} = numbers';
    
    textLine = fgets(fid);
    lineCounter = lineCounter + 1;
end

fclose(fid);

end

