function [R, T] = recoverPoseFromF(F, K, p1, p2, radius)
%recoverPoseFromF Recover R,T camera pose from the
% fundamental matrix F
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   F             Fundamental matrix
%   K             Intrinsics matrix
%   p1,p2      Inlier match
%   radius     Radius to project the sphere at
% Output
%   R,T          Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Obtain essential matrix
E = K'*F*K;

% Obtain R 
W = [0 -1 0; 1 0 0; 0 0 1];
[U,S,V] = svd(E);
R1 = U*W'*V';
R2 = U*W*V';

% Obtain T
T1 = U(:,3);
T2 = -T1;

% Convert to 3D
P1 = projectToSphere(K, p1, radius);
P2 = projectToSphere(K, p2, radius);

% Transform P1 into P2, check for positive depth
chosen = zeros(1, 4);
err = zeros(1, 4);
P2l = R1*P1 + T1; 
if P2l(3) > 0
    chosen(1) = 1;
    err(1) = norm(P2l - P2);
end
P2l = R1*P1 + T2; 
if P2l(3) > 0
    chosen(2) = 1;
    err(2) = norm(P2l - P2);
end
P2l = R2*P1 + T1; 
if P2l(3) > 0
    chosen(3) = 1;
    err(3) = norm(P2l - P2);
end
P2l = R2*P1 + T2; 
if P2l(3) > 0
    chosen(4) = 1;
    err(4) = norm(P2l - P2);
end

% Transform P2 into P1, check for positive depth
P1l = R1'*P2 - R1'*T1; 
if P1l(3) <= 0
    chosen(1) = 0;
end
P1l = R1'*P2 - R1'*T2; 
if P1l(3) <= 0
    chosen(2) = 0;
end
P1l = R2'*P2 - R2'*T1; 
if P1l(3) <= 0
    chosen(3) = 0;
end
P1l = R2'*P2 - R2'*T2; 
if P1l(3) <= 0
    chosen(4) = 0;
end

if sum(chosen) > 1
    aux = chosen.*err;
    aux(aux==0) = 100;
    [minErr, ch] = min(aux);
    chosen = zeros(1,4);
    chosen(ch) = 1;
end


if chosen(1)
    R = R1; T = T1;
else
    if chosen(2)
        R = R1; T = T2;
    else
        if chosen(3)
            R = R2; T = T1;
        else
            R = R2; T = T2;
        end
    end
end

end
