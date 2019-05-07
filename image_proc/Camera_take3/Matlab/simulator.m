function [M1, M2, m1, m2, err] = simulator(nMatches, R, maxD, minD, B, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulator Simulate two views of a 3D scene
% Input
%   nMatches Number of points
%   R              3D points rotation to make
%   maxD       Max distance to the camera
%   minD        Min distance to the camera
%   B              Baseline
%   K              Intrinsics matrix
% Output
%   M1,M2      3D points before and after
%   m1,m2     2D points before and after 
%                   transformation
%   err           Error of generation success
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

err = 0;
errorc = 0;
% Generate random 3D points
M1 = (maxD-minD)*rand([3, nMatches*10])+minD;
M1 = M1+B;
% Rotate points
M2 = R*M1;
% Keep the points with positive depth only
keep = M2(3, :) > 0;
M1 = M1(:, keep);
M2 = M2(:, keep);
% Guarantee there are at least N points with positive depth
while size(M1, 2) < nMatches
    M1 = (maxD-minD)*rand([3,N*10])+minD;
    M1 = M1+B;
    M2 = R*M1;
    keep = M2(3, :) > 0;
    M1 = M1(:, keep);
    M2 = M2(:, keep);
    errorc = errorc + 1;
    if errorc > 5 % try this max 5 times
        err = 1;
        break;
    end
end

if err ~= 1 
    M1 = M1(1:3, 1:nMatches); 
    M2 = M2(1:3, 1:nMatches); 
    
    m1 = projectToPlane(K, M1);
    m2 = projectToPlane(K, M2);
else
    m1 = 0; m2 = 0;
end

end

