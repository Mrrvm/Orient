close all;
clear;

I = [1 0 0; 0 1 0; 0 0 1];
N = 20;                      % number of point matches
maxd = 3;                    % max distance to camera
mind = 0.05;                 % min distance to camera
B = [0.0 0.0 0.07]';         % baseline
N_samples = 100;             % number of tries per axis
sigma = 5;                   % normal distribution sigma
angles = generateAngles(N_samples, sigma);

% x-1,y-2,z-3 
for i=1:3
    for j=1:N_samples
        %% Simulate points
        R = getRmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        [M1, M2, m1, m2, err] = pointGen(N, R, maxd, mind, B);
        if err == 1
            continue;
        end
        [m1, m2] = noiseGen(m1, m2, N);

        %% Run orthogonal procrustes problem
        [R_oppr, T_oppr] = orthProcrustesProb(m1, m2);
        
        %% Run matlab procrustes
        [R_fpro, T_fpro] = fullProcrustes(m1, m2, N);

        %% Run minimization of back projection error
        [R_mbpe, T_mbpe] = minBackProject(m1, m2, B, N, R_oppr);
        
        %% Run epipolar geometry approach
        [R_epog, T_epog] = epipolarGeo(m1, m2, I);

        %% Compute error between each method results and truth
        r       = matrixToAxisAngle(R);
        r_oppr  = matrixToAxisAngle(R_oppr);
        r_fpro  = matrixToAxisAngle(R_fpro);
        r_mbpe  = matrixToAxisAngle(R_mbpe);
        r_epog  = matrixToAxisAngle(R_epog);
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

fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meR_oppr(1), veR_oppr(1), meR_fpro(1), veR_fpro(1), meR_mbpe(1), veR_mbpe(1), meR_epog(1), veR_epog(1));
fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meR_oppr(2), veR_oppr(2), meR_fpro(2), veR_fpro(2), meR_mbpe(2), veR_mbpe(2), meR_epog(2), veR_epog(2));
fprintf('\n\t Mean | Variance \n OPPR | %f | %f \n FPRO | %f | %f \n MBPE | %f | %f \n EPOG | %f | %f \n\n', meR_oppr(3), veR_oppr(3), meR_fpro(3), veR_fpro(3), meR_mbpe(3), veR_mbpe(3), meR_epog(3), veR_epog(3));

%% Determine the method with least mean error
meR = [meR_oppr meR_fpro meR_mbpe meR_epog];
[minerror, method] = min(sum(meR));
label = {'error oppr', 'error fpro', 'error mbpe', 'error epog'};
fprintf('\nBest method was %s with %f half radians of error.\n\n', string(label(method)), minerror);

%% Visualise results
ang = angles(1:N_samples)*180/pi;
plotResults(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 1, 'x axis');
plotResults(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 2, 'y axis');
plotResults(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, 3, 'z axis');

clear;

