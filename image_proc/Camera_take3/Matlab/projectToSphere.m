function M = projectToSphere(K, m, radius)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project 2D coordinates into sphere
% Input
%   K          Intrinsics matrix
%   m         2D points 
%   radius  Sphere radius
% Output
%   M       3D points in sphere
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nMatches = size(m, 2);

mh = [m; ones(1, nMatches)];
mhk = K\mh;

x = mhk(1,:); y = mhk(2,:); z = mhk(3,:);
l = radius./sqrt(x.*x + y.*y + z.*z);

M = l.*mhk;
M(3,:) = l;

end
