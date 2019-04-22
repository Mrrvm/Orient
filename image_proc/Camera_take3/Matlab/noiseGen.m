function [mn1, mn2] = noiseGen(m1, m2, N, npixels)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate noise in pixel images
% Input
%   m1,m2    2D points before and after
%            transformation
%   npixels  Number of pixels to deviate 
%            from
% Output
%   mn1,mn2  2D points before and after
%            transformation with noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pixel2m = 0.0002645833;
max = pixel2m*npixels;
min = -pixel2m*npixels;
noise1 = (max-min)*rand([2, N])+min;
noise2 = (max-min)*rand([2, N])+min;

mn1 = m1 + noise1;
mn2 = m2 + noise2;

end