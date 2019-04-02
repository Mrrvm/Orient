function [M1, M2, m1, m2, err] = simulator(N, R, T, maxd, mind, B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% Rotate points
M2 = R*M1;
% Translate points
M2 = M2+T;
% Keep the points with positive depth only
keep = M2(3, :) > 0;
M1 = M1(:, keep);
M2 = M2(:, keep);
% Guarantee there are at least N points with positive depth
while size(M1, 2) < N 
    M1 = (maxd-mind)*rand([3,N*100])+mind;
    M2 = R*M1;
    M2 = M2+T;
    keep = M2(3, :) > 0;
    M1 = M1(:, keep);
    M2 = M2(:, keep);
    errorc = errorc +1;
    if errorc > 5
        err = 1;
        break;
    end
end

if err ~= 1 
    M1 = M1(1:3, 1:N); 
    M2 = M2(1:3, 1:N); 
    
    % Add baseline
    M1 = M1+B;
    M2 = M2+B;

    m1 = project_to_plane(M1);
    m2 = project_to_plane(M2);
else
    m1 = 0; m2 = 0;
end

end

