function [mn1, mn2] = falseMatchesGen(m1, m2, nFalseMatches, imgDim)
%falseMatches False Matches Generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate false matches in simulated data
% Input
%   m1,m2               2D points before and after
%                               transformation
%   nFalseMatches   Number of false point matches
%   imgDim              Image Dimensions 
% Output
%   mn1,mn2             2D points before and after
%                               transformation with false matches
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nMatches = size(m1, 2);
which = round((nMatches-1).*rand(nFalseMatches, 1))+1;
randx = round(imgDim(1).* rand(nFalseMatches,1));
randy = round(imgDim(2).* rand(nFalseMatches,1));
mn1 = m1;
mn2 = m2;

for i=1:nFalseMatches
    if rand() < 0.5
        mn1(1, which(i)) = randx(i);
        mn1(2, which(i)) = randy(i);
    else
        mn2(1, which(i)) = randx(i);
        mn2(2, which(i)) = randy(i);
    end  
end

end