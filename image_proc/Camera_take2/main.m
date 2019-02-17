%% SETTINGS
% Image resolution
width = 2056; height = 1542;
nobaseline = 1;
if nobaseline
    baseline = [0 0 0];
else
    % Example of baseline (must be measured)
    baselineIncm = [0 10 10];
    % Normalize baseline
    baseline = baselineIncm/(norm(baselineIncm));
end
I = [1 0 0;
     0 1 0;
     0 0 1];
% Camera Intrinsics (todo)
K = I;
fx = K(1,1);
fy = K(2,2);
cx = K(1,2);
cy = K(2,3);
s = K(1,2); %skew

%% SIMULATOR
% Number of points to generate
nPoints = 100;
% Rotation to impose (XYZ)
angles = [30 0 0];
% Radius of projection sphere
radius = 1;
% Maximum depth of scene objects
maxdepth = 1;
% Generate image points
[p1, p2, R, t] = PointGen(width, height, maxdepth, nPoints, angles, baseline, radius, K);

%% METHODS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Points in 3D to compute transformation for
P1 = SphereProj(p1, radius, 1, K);
P2 = SphereProj(p2, radius, 1, K);

%% 1 - PROCRUSTES ANALYSIS
[d,Z,tr] = procrustes(P1, P2, 'scaling', false, 'reflection', false); 
tProcrustes = tr.c(1,:);
rProcrustes =  I;

%% 2 - FUNDAMENTAL MATRIX ESTIMATION

%% 3 - PROCRUSTES ANALYSIS OPTIMIZATION WITH fminsearch (???) 

f = @(x)parameterfun(x, p1, p2, baseline, K)
x0 = [rProcrustes(1,1); rProcrustes(1,2); rProcrustes(1,3); 
    rProcrustes(2,1); rProcrustes(2,2); rProcrustes(2,3);
    rProcrustes(3,1); rProcrustes(3,2); rProcrustes(3,3);
    ones(nPoints, 1)];
[sol,fval,exitflag,output] = fminsearch(f, x0); 
%optimset('MaxFunEvals', nPoints*2)
rFmin = [sol(1) sol(2) sol(3); sol(4) sol(5) sol(6); sol(7) sol(8) sol(9)];


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




