function m = project_to_plane(M)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   M       3D points
% Output
%   m       2D points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m(1, :) = M(1,:)./M(3,:);
m(2, :) = M(2,:)./M(3,:);

end