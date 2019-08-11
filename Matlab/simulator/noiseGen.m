function [mn1, mn2] = noiseGen(m1, m2, nPixelsSigma, imgDim)
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

pd = makedist('normal', 0, nPixelsSigma);
nMatches = size(m1, 2);
iters = 0;
maxIters = 10;

mn1 = m1;
mn2 = m2;

for i=1:nMatches
    
    iters = 0;
    rand = pd.random;
    while( ~( (m1(1, i) + rand < imgDim(1)) && (m1(1, i) + rand > 0) ) && (iters < maxIters ))
        rand = pd.random;
        iters = iters +1;
    end
    if iters < maxIters
        mn1(1, i) = mn1(1, i) + rand;
    end
    
    iters = 0;
    rand = pd.random;
    while( ~( (m1(2, i) + rand < imgDim(2)) && (m1(2, i) + rand > 0) ) && (iters < maxIters ))
       rand = pd.random;
       iters = iters +1;
    end
    if iters < maxIters
        mn1(2, i) = mn1(2, i) + rand;
    end
    
    iters = 0;
    rand = pd.random;
    while( ~( (m2(1, i) + rand < imgDim(1)) && (m2(1, i) + rand > 0) ) && (iters < maxIters ))
        rand = pd.random;
        iters = iters +1;
    end
    if iters < maxIters
        mn2(1, i) = mn2(1, i) + rand;
    end
    
    iters = 0;
    rand = pd.random;
    while( ~( (m2(2, i) + rand < imgDim(2)) && (m2(2, i) + rand > 0) ) && (iters < maxIters ))
        rand = pd.random;
        iters = iters +1;
    end
    if iters < maxIters
        mn2(2, i) = mn2(2, i) + rand;
    end
    
end

end