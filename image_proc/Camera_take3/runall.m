close all;
clear all;

I = [1 0 0; 0 1 0; 0 0 1];
N = 10;                      % number of point matches
d = 1;                      % distance to camera
B = [0.0 0.1 0.0]';         % baseline
N_samples = 10;             % number of tries per axis
sigma = 5;                  % normal distribution sigma
angles = generate_angles(N_samples, sigma);

% x-1,y-2,z-3 
for i=1:3
    for j=1:N_samples
        %% Simulate points
        R = get_Rmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        [M1, M2, m1, m2] = simulator(N, d, R, T);

        %% Run orthogonal procrustes problem
        [R_pro, T_pro] = opprocrustes(m1, m2);

        %% Run minimization of back projection error
        [R_mbpe, T_mbpe] = minbackproject(m1, m2, B, N, R_pro);

        %% Compare methods
        r      = matrix_to_axisangle(R);
        r_pro  = matrix_to_axisangle(R_pro);
        r_mbpe = matrix_to_axisangle(R_mbpe);
        eR_pro(i,j)  = norm(r-r_pro);
        eR_mbpe(i,j) = norm(r-r_mbpe);
        eT_pro(i,j)  = norm(T-T_pro);
        eT_mbpe(i,j) = norm(T-T_mbpe);
    end  
end

%% Visualise results
ang = angles(1:N_samples)*180/pi;

figure;
plot(ang, eR_pro(1,:), 'bo');
hold on;
plot(ang, eR_mbpe(1,:), 'r+');
hold on;
coeffs_pro = polyfit(ang, eR_pro(1,:), 1);
coeffs_mbpe = polyfit(ang, eR_mbpe(1,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_pro = polyval(coeffs_pro, fittedX);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_pro, 'b-', 'LineWidth', 3);
hold on;
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 3);
title('x axis');
legend({'erro pro','erro mbpe'},'Location','southwest');
xlabel('Degrees');
ylabel('Half Radians');

figure;
plot(ang, eR_pro(2,:), 'bo');
hold on;
plot(ang, eR_mbpe(2,:), 'r+');
hold on;
coeffs_pro = polyfit(ang, eR_pro(2,:), 1);
coeffs_mbpe = polyfit(ang, eR_mbpe(2,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_pro = polyval(coeffs_pro, fittedX);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_pro, 'b-', 'LineWidth', 3);
hold on;
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 3);
title('y axis');
legend({'erro pro','erro mbpe'},'Location','southwest');
xlabel('Degrees');
ylabel('Half Radians');

figure;
plot(ang, eR_pro(3,:), 'bo');
hold on;
plot(ang, eR_mbpe(3,:), 'r+');
hold on;
coeffs_pro = polyfit(ang, eR_pro(3,:), 1);
coeffs_mbpe = polyfit(ang, eR_mbpe(3,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_pro = polyval(coeffs_pro, fittedX);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_pro, 'b-', 'LineWidth', 3);
hold on;
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 3);
title('z axis');
legend({'erro pro','erro mbpe'},'Location','southwest');
xlabel('Degrees');
ylabel('Half Radians');


