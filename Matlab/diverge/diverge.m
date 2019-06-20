clear all; close all;

I = [1 0 0; 0 1 0; 0 0 1];
axisOffset = [0 0];
focalLength = 1;
m2pix = [3779.53 3779.53];     
skew = 0;
K =  [focalLength*m2pix(1)   skew                                axisOffset(1); 
         0                                   focalLength*m2pix(2)     axisOffset(2); 
         0                                   0                                     1                 ];
K = I;
nPixels = 0;
B = [0 0 1]';
sigma = 5;
maxD = 2;
minD = 1.50;
radius = 1;

angles = -45*pi/180;%[40 35 30 25 20 15 10 5 4 3 2 1]'*pi/180;

nAngles = size(angles, 1);
% Generate random angles in normal distribution
%angles = generateAngles(nAngles, sigma);   % Comment this to keep the same angles

% Generate random 3D points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Comment this to keep the same M1 points
%Mw = (maxD-minD)*rand([3, 10])+minD;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Mw = [ 0,  0,  2;    
            0,  2 , 0;
            2,  2,  2];
       
nMatches = size(Mw, 2);
       
for i=1:3
    for j=1:nAngles
        % Generate rotation and translation matrix
        R = eul2rotm(-angles(j)*[(3-i)==0 (2-i)==0 (1-i)==0]);
        T = (R-I)*B;
        % Simulate points
        [M1, M2, m1, m2] = constant_simulator(Mw, nMatches, R, T, maxD, minD, B, K);
        % Initializtion from Procrustes
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        % Minimization back projection error
        [Rmbpe, Tmbpe] = minBackProject(m1, m2, B, rotm2eul(Roppr), radius, K);
        % Determine error compared to ground truth
        eRmbpe(i,j) = norm(rotm2eul(Rmbpe)-rotm2eul(R));
        eRoppr(i,j) = norm(rotm2eul(Roppr)-rotm2eul(R));
    end  
end

% Plot results
eRmbpe = eRmbpe*180/pi;
ang = angles(1:nAngles)'*180/pi;
label = {'x error', 'y error', 'z error'};
fig = figure;
plot(ang , eRmbpe(1,:), 'bo');
hold on;
plot(ang, eRmbpe(2,:), 'ro');
hold on;
plot(ang, eRmbpe(3,:), 'go');
hold on;
coeffs_mbpe = polyfit(ang, eRmbpe(1,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_mbpe, 'b-', 'LineWidth', 1);
hold on;
coeffs_mbpe = polyfit(ang, eRmbpe(2,:), 1);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 1);
hold on;
coeffs_mbpe = polyfit(ang, eRmbpe(3,:), 1);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
plot(fittedX, fittedY_mbpe, 'g-', 'LineWidth', 1);
hold on;
title('Error per angle (MBPE)');
legend(label,'Location','northeast');
xlabel('Angles (degrees)');
ylabel('Error (degrees)');

eRoppr = eRoppr*180/pi;
ang = angles(1:nAngles)'*180/pi;
label = {'x error', 'y error', 'z error'};
fig = figure;
plot(ang , eRoppr(1,:), 'bo');
hold on;
plot(ang, eRoppr(2,:), 'ro');
hold on;
plot(ang, eRoppr(3,:), 'go');
hold on;
coeffs_oppr = polyfit(ang, eRoppr(1,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_oppr = polyval(coeffs_oppr, fittedX);
plot(fittedX, fittedY_oppr, 'b-', 'LineWidth', 1);
hold on;
coeffs_oppr = polyfit(ang, eRoppr(2,:), 1);
fittedY_oppr = polyval(coeffs_oppr, fittedX);
plot(fittedX, fittedY_oppr, 'r-', 'LineWidth', 1);
hold on;
coeffs_oppr = polyfit(ang, eRoppr(3,:), 1);
fittedY_oppr = polyval(coeffs_oppr, fittedX);
plot(fittedX, fittedY_oppr, 'g-', 'LineWidth', 1);
hold on;
title('Error per angle (Procrustes)');
legend(label,'Location','northeast');
xlabel('Angles (degrees)');
ylabel('Error (degrees)');



