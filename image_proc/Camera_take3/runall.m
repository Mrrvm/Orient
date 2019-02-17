%% Simulate points
N = 10; % number of point matches
d = 4;  % distance to camera
R = get_Rmatrix(45, 0, 0);
T = [0 0 0]';
[m1, m2] = simulator(N, d, R, T);

%% Run orthogonal procrustes problem
[R_pro, T_pro] = opprocrustes(m1, m2);

%% Run minimization of back projection error
[R_mbpe, T_mbpe] = minbackproject(m1, m2);