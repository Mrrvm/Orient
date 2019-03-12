function [p1, p2, R, t] = PointGen(width, height, maxdepth, nPoints, angles, t, radius, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Returns two vectors of (x,y) pixels p1 and p2 resultant of a 
%      rotation R and translation t, which is also returned.
%
%  In
%      width    :  Width of the image
%      height   : Height of the image
%      maxdepth : Ma
%      nPoints  : Number of points to generate
%      angles   : Vector of radians representing what rotation matrix to 
%                 generate [rx ry rz]
%      t        : Vector of distances representing what translation to 
%                 generate [tx ty tz]
%      radius   : Radius of sphere projection
%      K        : 3x3 matrix with camera's intrinsic parameters
%  Out
%      p1       : Points before rotation/translation
%      p2       : Points after rotation/translation
%      R        : Rotation matrix 
%      t        : Translation vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,2);
    cy = K(2,3);
    s  = K(1,2);
    
    % Generate random keypoints on image 1
    p1       = zeros(nPoints, 2);
    p1(:, 1) = (width).*rand(nPoints,1);
    p1(:, 2) = (height).*rand(nPoints,1);
    lambda   = (maxdepth).*rand(nPoints,1);
    
    % Get p1 in 3D 
    P1       = zeros(nPoints, 3);
    P1(:, 3) = lambda(:);
    P1(:, 1) = (((P1(:, 3).*p1(:, 1))-cx).*P1(:, 3))/fx;
    P1(:, 2) = (((P1(:, 3).*p1(:, 2))-cy).*P1(:, 3))/fy;
    
    % Graphics
    figure(1);
    title("Image 1 keypoints");
    plot(p1(:,1), p1(:,2), 'bx');
    figure(2);  
    title("Image 1 3D keypoints");
    plot3(P1(:,1), P1(:,2), P1(:,3),'bx');
    
    % Generate rotation matrix
    R = eul2rotm([angles(1) angles(2) angles(3)], 'ZYX'); 
    
    % Rotate and translate to get P2
    P2 = (R*P1' + t')';
    
    % Get matching keypoints on image 2
    p2 = zeros(nPoints, 2);
    p2(:, 1) = (((fx*P2(:,1))./P2(:, 3))+cx)./P2(:, 3);
    p2(:, 2) = (((fy*P2(:,2))./P2(:, 3))+cy)./P2(:, 3);

    % Graphics
    figure(3);
    title("Image 2 keypoints");
    plot(p2(:,1), p2(:,2), 'bx');
    figure(4);
    title("Image 2 3D keypoints");
    plot3(P2(:,1), P2(:,2), P2(:,3),'bx');
    
end

