function [mn1, mn2] = noiseGen(m1, m2, nMatches, nPixels)
%noiseGen Pixel Noise Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate noise in pixel images
% Input
%   m1,m2      2D points before and after
%              transformation
%   nMatches   Number of point matches
%   nPixels    Number of pixels to deviate 
% Output
%   mn1,mn2    2D points before and after
%              transformation with noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

max =  nPixels;
min = -nPixels;
noise1 = (max-min)*rand([2, nMatches])+min;
noise2 = (max-min)*rand([2, nMatches])+min;

mn1 = m1 + noise1;
mn2 = m2 + noise2;

end