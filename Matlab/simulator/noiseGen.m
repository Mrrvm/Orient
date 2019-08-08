function [mn1, mn2] = noiseGen(m1, m2, nPixels, imgDim)
%noiseGen Pixel Noise Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate noise in pixel images
% Input
%   m1,m2      2D points before and after
%              transformation
%   nMatches   Number of point matches
%   nPixels    Number of pixels to deviate 
%   imgDim   Image dimensions
% Output
%   mn1,mn2    2D points before and after
%              transformation with noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pd = makedist('normal', 0, nPixels/3);
nMatches = size(m1, 2);

for i=1:nMatches
    
    rand = pd.random;
    
    while( ~( (m1(1, i) + rand < imgDim(1)) && (m1(1, i) + rand > 0) ) )
        rand = pd.random;
    end
     mn1(1, i) = m1(1, i) + rand;
     
    while( ~( (m1(2, i) + rand < imgDim(2)) && (m1(2, i) + rand > 0) ) )
       rand = pd.random;
    end
    mn1(2, i) = m1(2, i) + rand;
    
    while( ~( (m2(1, i) + rand < imgDim(1)) && (m2(1, i) + rand > 0) ) )
        rand = pd.random;
    end
    mn2(1, i) = m2(1, i) + rand;
    
    while( ~( (m2(2, i) + rand < imgDim(2)) && (m2(2, i) + rand > 0) ) )
        rand = pd.random;
    end
    mn2(2, i) = m2(2, i) + rand;
    
end

end