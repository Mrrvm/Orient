function [m, Z] = projectToPlane(K, M)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project 3D coordinates into plane
% Input
%   M       3D points
%   K        Intrinsics matrix
% Output
%   m       2D points 
%   Z        Depth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nMatches = size(M, 2);
mh(1, :) = M(1,:)./M(3,:);
mh(2, :) = M(2,:)./M(3,:);
mh(3,:) = ones(1, nMatches);
mh = K*mh;
m = mh(1:2, :);
Z = M(3,:);


end