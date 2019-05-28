function [M1, M2, m1, m2, err] = constant_simulator(M1, nMatches, R, maxD, minD, B, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulator Simulate two views of a 3D scene
% Input
%   nMatches Number of points
%   R              3D points rotation to make
%   maxD       Max distance to the camera
%   minD        Min distance to the camera
%   B              Baseline
%   K              Intrinsics matrix
%   M1           3D points previously generated
%                       pre-transformation
% Output
%   M2           3D points pos transformation
%   m1,m2     2D points pre and pos-transformation
%   err           Error of generation success
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

err = 0;
errorc = 0;

% Rotate points
M2 = R*M1;
% Keep the points with positive depth only
keep = M2(3, :) > 0;
M1 = M1(:, keep);
M2 = M2(:, keep);
% Guarantee there are at least nMatches points with positive depth
while size(M1, 2) < nMatches
    M1 = (maxD-minD)*rand([3, nMatches*10])+minD;
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

