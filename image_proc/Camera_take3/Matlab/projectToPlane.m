function m = projectToPlane(M)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project 3D coordinates into plane
% Input
%   M       3D points
% Output
%   m       2D points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Mk = M;
m(1, :) = M(1,:)./M(3,:);
m(2, :) = M(2,:)./M(3,:);

end