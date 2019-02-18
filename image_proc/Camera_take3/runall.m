I = [1 0 0; 0 1 0; 0 0 1];

%% Simulate points
N = 10;                   % number of point matches
d = 4;                      % distance to camera
B = [0 0 0.05]';            % baseline
R = get_Rmatrix(0, 0, 0);
T = (R-I)*B;
[m1, m2] = simulator(N, d, R, T);

%% Run orthogonal procrustes problem
[R_pro, T_pro] = opprocrustes(m1, m2);

%% Run minimization of back projection error
[R_mbpe, T_mbpe] = minbackproject(m1, m2, B, N);