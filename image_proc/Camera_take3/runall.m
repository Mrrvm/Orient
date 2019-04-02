close all;
clear;

I = [1 0 0; 0 1 0; 0 0 1];
N = 20;                     % number of point matches
maxd = 0.5;                 % max distance to camera
mind = 0.05;                % min distance to camera
B = [0.0 0.0 0.7]';         % baseline
N_samples = 10;             % number of tries per axis
sigma = 5;                  % normal distribution sigma
angles = generate_angles(N_samples, sigma);

% x-1,y-2,z-3 
for i=1:3
    for j=1:N_samples
        %% Simulate points
        R = get_Rmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        [M1, M2, m1, m2, err] = simulator(N, R, T, maxd, mind, B);
        if err == 1
            continue;
        end
        [m1, m2] = noisegen(m1, m2, N);

        %% Run orthogonal procrustes problem
        [R_oppr, T_oppr] = opprocrustes(m1, m2);
        
        %% Run matlab procrustes
        [R_fpro, T_fpro] = fullprocrustes(m1, m2, N);

        %% Run minimization of back projection error
        [R_mbpe, T_mbpe] = minbackproject(m1, m2, B, N, R_fpro);
        
        %% Run epipolar geometry approach
        [R_epog, T_epog] = epipolargeo(m1, m2, I);

        %% Compute error between each method results and truth
        r       = matrix_to_axisangle(R);
        r_oppr  = matrix_to_axisangle(R_oppr);
        r_fpro  = matrix_to_axisangle(R_fpro);
        r_mbpe  = matrix_to_axisangle(R_mbpe);
        r_epog  = matrix_to_axisangle(R_epog);
        eR_oppr(i,j)  = norm(r-r_oppr);
        eR_fpro(i,j)  = norm(r-r_fpro);
        eR_mbpe(i,j)  = norm(r-r_mbpe);
        eR_epog(i,j)  = norm(r-r_epog);
        eT_oppr(i,j)  = norm(T-T_oppr);
        eT_fpro(i,j)  = norm(T-T_fpro);
        eT_mbpe(i,j)  = norm(T-T_mbpe);
        eT_epog(i,j)  = norm(T-T_epog);
    end  
end

%% Compute mean error and variance
meT_oppr = mean(eT_oppr, 2);
meT_fpro = mean(eT_fpro, 2);
meT_mbpe = mean(eT_mbpe, 2);
meT_epog = mean(eT_epog, 2);

meR_oppr = mean(eR_oppr, 2);
meR_fpro = mean(eR_fpro, 2);
meR_mbpe = mean(eR_mbpe, 2);
meR_epog = mean(eR_epog, 2);

veR_oppr = sum((eR_oppr - meR_oppr).^2,2)/N;
veR_fpro = sum((eR_fpro - meR_fpro).^2,2)/N;
veR_mbpe = sum((eR_mbpe - meR_mbpe).^2,2)/N;
veR_epog = sum((eR_epog - meR_epog).^2,2)/N;

%% Determine the method with least mean error
meR = [meR_oppr meR_fpro meR_mbpe meR_epog];
[minerror, method] = min(sum(meR));
label = {'error oppr', 'error fpro', 'error mbpe', 'error epog'};
fprintf('\nBest method was %s with %f half radians of error.\n\n', string(label(method)), minerror);

%% Visualise results
ang = angles(1:N_samples)*180/pi;
plotgen(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 1);
plotgen(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 2);
plotgen(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 3);


