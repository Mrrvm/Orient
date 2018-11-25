function [p1, p2, R, t] = PointGen(width, height, nPoints, rType, tType, radius)
%  Returns two vectors of (x,y) pixels p1 and p2 resultant of a 
%      rotation R and translation t, which is also returned.
%      The points are projected in a sphere before rotation/translation.
%
%  In:
%      width:   Width of the image
%      height:  Height of the image
%      nPoints: Number of points to generate
%      rType:   Vector of degrees representing what rotation matrix to 
%               generate [rx ry rz]
%      tType:   Vector of distances representing what translation to 
%               generate [tx ty tz]
%      radius:  Radius of sphere projection
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
    
    P1 = SphereProj(p1, radius, 1);
    
    P2 = R*P1 + t;
    
    p2 = SphereProj(P2, radius, 0);

end


