    function [R, T] = epipolarGeo(m1, m2, radius, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimate transformation using epipolar 
% geometry
% Input
%   m1,m2    2D points before and after
%            transformation
%   radius   Sphere radius
%   K        Intrinsics matrix
% Output
%   R,T      Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kt = K';

% Estimates fundamental matrix using LMedS
%M1 = projectToSphere(m1, radius);
%M2 = projectToSphere(m2, radius);
F = estimateFundamentalMatrix(m1', m2');

% Obtain essential matrix
E = Kt*F*K;

% Obtain R 
W = [0 -1 0; 1 0 0; 0 0 1];
[U,S,V] = svd(E);
R1 = U*W'*V';
R2 = U*W*V';

% Obtain T
T1 = U(:,3);
T2 = -T1;

% Get one 2D match
p1 = [m1(:,1); 1];
p2 = [m2(:,1); 1];

% Convert to 3D
P1 = projectToSphere(p1, radius);
P1 = Kt*P1;

% Transform the point and compare with original
P2l = R1*P1 + T1; 
p2l = [projectToPlane(K*P2l); 1];
err(1) = norm(p2 - p2l);

P2l = R1*P1 + T2; 
p2l = [projectToPlane(K*P2l); 1];
err(2) = norm(p2 - p2l);

P2l = R2*P1 + T1; 
p2l = [projectToPlane(K*P2l); 1];
err(3) = norm(p2 - p2l);

P2l = R2*P1 + T2; 
p2l = [projectToPlane(K*P2l); 1];
err(4) = norm(p2 - p2l);

% Choose the transformation with least error
[a chosen] = min(err);

if chosen == 1
    R = R1; T = T1;
else
    if chosen == 2
        R = R1; T = T2;
    else
        if chosen == 3
            R = R2; T = T1;
        else
            R = R2; T = T2;
        end
    end
end
    
end
