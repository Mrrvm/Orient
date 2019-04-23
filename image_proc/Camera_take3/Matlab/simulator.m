function [eRoppr, eRfpro, eRmbpe, eRepog] = simulator(angles, radius, K, nMatches, maxD, minD, B, nAngles, nPixels)
%simulator Simulate methods of computing rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate points on space and test methods
% Input
%   angles     Set of angles to test
%   radius     Sphere radius
%   K          Intrinsics matrix
%   nMatches   Number of matches per sample
%   maxD       Max distance to camera
%   minD       Min distance to camera
%   B          Baseline
%   nAngles    Number of different angles to try
%   nPixels    Number of pixels to deviate in noise
% Output
%   eR...      Error from each method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [1 0 0; 0 1 0; 0 0 1];

% x-1,y-2,z-3 
for i=1:3
    for j=1:nAngles
        %% Simulate points
        R = getRmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        [M1, M2, m1, m2, err] = pointGen(nMatches, R, maxD, minD, B, K);
        if err == 1
            continue;
        end
        %showScenario(M1, M2, B, R, maxD);
        [m1, m2] = noiseGen(m1, m2, nMatches, nPixels);

        %% Run orthogonal procrustes problem
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        
        %% Run matlab procrustes
        [Rfpro, Tfpro] = fullProcrustes(m1, m2, radius, K);

        %% Run minimization of back projection error
        [Rmbpe, Tmbpe] = minBackProject(m1, m2, B, nMatches, Roppr, radius, K);
        
        %% Run epipolar geometry approach
        [Repog, Tepog] = epipolarGeo(m1, m2, radius, K);

        %% Compute error between each method results and truth
        r      = matrixToAxisAngle(R);
        roppr  = matrixToAxisAngle(Roppr);
        rfpro  = matrixToAxisAngle(Rfpro);
        rmbpe  = matrixToAxisAngle(Rmbpe);
        repog  = matrixToAxisAngle(Repog);
        eRoppr(i,j)  = norm(r-roppr);
        eRfpro(i,j)  = norm(r-rfpro);
        eRmbpe(i,j)  = norm(r-rmbpe);
        eRepog(i,j)  = norm(r-repog);
        eToppr(i,j)  = norm(T-Toppr);
        eTfpro(i,j)  = norm(T-Tfpro);
        eTmbpe(i,j)  = norm(T-Tmbpe);
        eTepog(i,j)  = norm(T-Tepog);
    end  
end
 
end