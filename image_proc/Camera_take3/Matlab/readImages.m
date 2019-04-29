function [imgs1, imgs2, axisCount] = readImages(imgDir)
%readImages Read images with name im[1/2][x/y/z][angle in degrees].jpg
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read images into useful data
% Input
%   imgDir       Image directory to read from
% Output
%   imgs1,       Data with grayscale images, axis, angle and 
%   imgs2        distance information
%   axisCount  Number of images per axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

axisCount = zeros(1,3);

files1 = dir(fullfile(imgDir,'im1*'));
files2 = dir(fullfile(imgDir,'im2*'));

for k = 1:numel(files1) 
    name = files1(k).name;
    img = fullfile(imgDir, name);
    grayimg = rgb2gray(imread(img));
    imgs1(k).data = grayimg; 
    
    imgs1(k).axis = name(4);
    imgs2(k).axis = imgs1(k).axis;
    [token remain] = strtok(name(5:end), '.');
    imgs1(k).angle = str2double(token);
    imgs2(k).angle = imgs1(k).angle;
    
    name = files2(k).name;
    img = fullfile(imgDir, name);
    grayimg = rgb2gray(imread(img));
    imgs2(k).data = grayimg; 
    
    if name(4) == 'x'
        axisCount(1) = axisCount(1) + 1;
    else
        if name(4) == 'y'
            axisCount(2) = axisCount(2) + 1;
        else
            if name(4) == 'z'
                axisCount(3) = axisCount(3) + 1;
            end
        end
    end
    
end



