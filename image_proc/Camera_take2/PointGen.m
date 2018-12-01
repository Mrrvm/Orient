function [p1, p2, R, t] = PointGen(width, height, nPoints, angles, t, radius, K)
%  Returns two vectors of (x,y) pixels p1 and p2 resultant of a 
%      rotation R and translation t, which is also returned.
%      The points are projected in a sphere before rotation/translation.
%
%  In:
%      width:   Width of the image
%      height:  Height of the image
%      nPoints: Number of points to generate
%      angles:  Vector of radians representing what rotation matrix to 
%               generate [rx ry rz]
%      t:       Vector of distances representing what translation to 
%               generate [tx ty tz]
%      radius:  Radius of sphere projection
%      K: 3x3 matrix with camera's intrinsic parameters
%  Out:
%      p1:      Points before rotation/translation
%      p2:      Points after rotation/translation
%      R:       Rotation matrix 
%      t:       Translation vector

    p1 =  zeros(nPoints, 2);
    min = 0;
    max = width;
    p1(:, 1) = (max-min).*rand(nPoints,1) + min;
    max = height;
    p1(:, 2) = (max-min).*rand(nPoints,1) + min;
    
    P1 = SphereProj(p1, radius, 1, K);
    
    R = eul2rotm([angles(1) angles(2) angles(3)], 'ZYX'); 
    
    P2 = R*P1' + t';
    
    p2 = SphereProj(P2', radius, 0, K);

end


