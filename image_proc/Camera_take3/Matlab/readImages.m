function [imgs1, imgs2] = readImages(imgdir)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read images into useful data
% Input
%   imgdir   Image dir to read from
% Output
%   imgs1,   Data with grayscale images
%   imgs2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imgs1 = dir(fullfile(imgdir,'img1*.jpg'));
imgs2 = dir(fullfile(imgdir,'img2*.jpg'));
for k = 1:numel(imgs1)
    img1 = fullfile(imgdir, imgs1(k).name);
    gray_img1 = rgb2gray(imread(img1));
    imgs1(k).data = gray_img1; 
    img2 = fullfile(imgdir, imgs2(k).name);
    gray_img2 = rgb2gray(imread(img2));
    imgs2(k).data = gray_img2; 
end