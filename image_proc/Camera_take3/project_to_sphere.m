function M = project_to_sphere(m, radius)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m       2D points 
%   radius  Sphere radius
% Output
%   M       3D points in sphere
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine lambda as ||(?x, ?y, ?)|| = 1
x = m(1,:); y = m(2,:);
l = radius./sqrt( 1 + x.*x + y.*y);

M = l.*m;
M(3,:) = l;

end
