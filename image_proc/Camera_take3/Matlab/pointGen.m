function [M1, M2, m1, m2, err] = pointGen(N, R, maxd, mind, B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a 3D points transformation
% Input
%   angles  Rotation angles per axis
%   T       Translation
%   N       Number of points
%   d       distance from the camera
% Output
%   m1,m2   2D points before and after 
%            transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

err = 0;
errorc = 0;
% Generate random 3D points
M1 = (maxd-mind)*rand([3,N*100])+mind;
M1 = M1+B;
% Rotate points
M2 = R*M1;
% Keep the points with positive depth only
keep = M2(3, :) > 0;
M1 = M1(:, keep);
M2 = M2(:, keep);
% Guarantee there are at least N points with positive depth
while size(M1, 2) < N 
    M1 = (maxd-mind)*rand([3,N*100])+mind;
    M1 = M1+B;
    M2 = R*M1;
    keep = M2(3, :) > 0;
    M1 = M1(:, keep);
    M2 = M2(:, keep);
    errorc = errorc +1;
    if errorc > 5 % try this max 5 times
        err = 1;
        break;
    end
end

if err ~= 1 
    M1 = M1(1:3, 1:N); 
    M2 = M2(1:3, 1:N); 
    
    m1 = projectToPlane(M1);
    m2 = projectToPlane(M2);
else
    m1 = 0; m2 = 0;
end

%showScenario(M1, M2, B, R, maxd);

end

