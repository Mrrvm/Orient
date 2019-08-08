function [Mw, M1, M2, m1, m2, err] = simulator(nMatches, R, T, maxD, minD, B, K, imgDim)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%simulator Simulate two views of a 3D scene
% Input
%   nMatches Number of points
%   R              3D points rotation to make
%   maxD       Max distance to the points
%   minD        Min distance to the points
%   B              Baseline
%   K              Intrinsics matrix
% Output
%   M1,M2      3D points before and after
%   m1,m2     2D points before and after 
%                   transformation
%   err           Error of generation success
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

err = 0;
trials = 100;

sz = 1;
Z = (maxD-minD)*rand(1,nMatches*trials)+minD;
for i=1:trials*nMatches
    
    Y = 2*Z(i)*rand(1,1)-Z(i);
    X = 2*Z(i)*rand(1,1)-Z(i);
    Mwi = [X Y Z(i)]';
    M1i = Mwi-B;
    M2i = R*M1i+T;
    m1i = projectToPlane(K, M1i);
    m2i = projectToPlane(K, M2i);

    if ((m1i(1) <= imgDim(1)) && (m1i(1) >= 0) && (m1i(2) <= imgDim(2)) && (m1i(2) >= 0) && ...
            (m2i(1) <= imgDim(1)) && (m2i(1) >= 0) && (m2i(2) <= imgDim(2)) && (m2i(2) >=0))
        Mw(:, sz) = Mwi;
        M1(:, sz) = M1i;
        M2(:, sz) = M2i;
        m1(:, sz) = round(m1i);
        m2(:, sz) = round(m2i);
        if sz == nMatches
            break;
        end
        sz = sz + 1;
    end
    
end

if sz < nMatches
    err = 1;
    return;
end

end

