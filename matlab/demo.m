image = imread('D:\Pictures\2Dto3D\IMG_2701.JPG');
image = imresize(image, 1024 / size(image,2));
figure
hold on
imshow(image);

% for j=0:3:3
%     path = ['..\lines_' num2str(j) '.csv'];
%     lines = load(path);
%     for i = 1:size(lines, 1)
%         line([lines(i,1), lines(i,3)], [lines(i,2), lines(i,4)], 'LineWidth', 1.0, 'Color', [1,0,0]);
%     end
% end

lines = load('..\lines_0.csv');
for i = 1:size(lines, 1)
    line([lines(i,1), lines(i,3)], [lines(i,2), lines(i,4)], 'LineWidth', 1.0, 'Color', [1,0,0]);
end