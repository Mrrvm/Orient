%% SETTINGS
% Image resolution
width = 2056; height = 1542;
% Example of baseline (must be measured)
baselineIncm = [0 10 10];
% Normalize baseline
baseline = baselineIncm/(norm(baselineIncm));
% Camera Intrinsics (to change)
K = [1 0 0;
     0 1 0;
     0 0 1];

%% SIMULATOR
% Number of points to generate
nPoints = 100;
% Rotation to impose (XYZ)
angles = [0 0 0];
% Radius of projection sphere
radius = 1;
% Generate image points
[p1, p2, R, t] = PointGen(width, height, nPoints, angles, baseline, radius, K);

%% METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Points in 3D to compute transformation for
P1 = SphereProj(p1, radius, 1, K);
P2 = SphereProj(p2, radius, 1, K);

%% 1 - PROCRUSTES ANALYSIS
[d,Z,tr] = procrustes(P1,P2,'scaling', false, 'reflection', false); 
tProcrustes = tr.c(1,:);
rProcrustes = tr.T;

%% 2 - FUNDAMENTAL MATRIX ESTIMATION

%% 3 - PROCRUSTES ANALYSIS OPTIMIZATION WITH fminsearch (???) 

f = @(rFmin)square(norm( P2 - (rFmin*P1' + (rFmin*baseline'-baseline') )' , 2));
r0 = rProcrustes;
[rFmin,fval,exitflag,output] = fminsearch(f, r0); %optimset('MaxFunEvals', nPoints*2)


%% 4 - CVX

cvx_begin quiet
    variable rCVX(3,3);
    
    fCVX = 0;
    for i=1:nPoints
        fCVX = fCVX + square_pos(norm(P2(i,:) - ((rCVX*P1(i,:)')' + t), 2));
    end
    
    minimize(fCVX);
    
    % subject to
    t = rCVX*baseline'-baseline';

cvx_end




