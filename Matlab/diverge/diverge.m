I = [1 0 0; 0 1 0; 0 0 1];
axisOffset = [0 0];
focalLength = 1;
m2pix = [3779.53 3779.53];     
skew = 0;
K =  [focalLength*m2pix(1)   skew                                axisOffset(1); 
         0                                   focalLength*m2pix(2)     axisOffset(2); 
         0                                   0                                     1                 ];
%K = I;
nAngles = 20;
nPixels = 0;
B = [0.02 -0.055 0.055]';
sigma = 5;
maxD = 5;
minD = 0.05;
radius = 1;
nMatches = 10;

% Generate random angles in normal distribution
angles = generateAngles(nAngles, sigma);   % Comment this to keep the same angles

% Generate random 3D points 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Comment this to keep the same M1 points
if maxD == minD
    a = rand([2, nMatches*10])+minD;
    M1(1:2, :) = a;
    M1(3, :) = ones(1, nMatches*10)*minD;
else
    M1 = (maxD-minD)*rand([3, nMatches*10])+minD;
end
M1 = M1+B;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for i=1:3
    for j=1:nAngles
        % Generate rotation and translation matrix
        R = getRmatrix(angles((i-1)*j+j,:));
        T = (R-I)*B;
        % Simulate points
        [M1, M2, m1, m2, err] = constant_simulator(M1, nMatches, R, maxD, minD, B, K);
        if err == 1
            continue;
        end
        %showScenario(M1, M2, B, R, maxD);
        %[m1, m2] = noiseGen(m1, m2, nMatches, nPixels);
        r = matrixToAxisAngle(R);
        % Initializtion from Procrustes
        [Roppr, Toppr] = orthProcrustesProb(m1, m2, radius, K);
        % Minimization back projection error
        [Rmbpe, Tmbpe] = minBackProject(m1, m2, B, Roppr, radius, K);
        % Determine error compared to ground truth
        rmbpe = matrixToAxisAngle(Rmbpe);
        eRmbpe(i,j) = norm(r-rmbpe);
    end  
end

% Plot results
ang = angles(1:nAngles)*180/pi;
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
title('Error per angle');
legend(label,'Location','northeast');
xlabel('Angles (degrees)');
ylabel('Error (rad/2)');



