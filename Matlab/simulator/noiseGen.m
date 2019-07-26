function [mn1, mn2] = noiseGen(m1, m2, nPixels)
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

pd = makedist('normal', 0, nPixels/3);
nMatches = size(m1, 2);

for i=1:nMatches
    mn1(1, i) = m1(1, i) + pd.random;
    mn1(2, i) = m1(2, i) + pd.random;
    mn2(1, i) = m2(1, i) + pd.random;
    mn2(2, i) = m2(2, i) + pd.random;
end

end