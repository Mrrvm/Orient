function plotResults(ang, eR_oppr, eR_fpro, eR_mbpe, eR_epog, i, axis)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot results with a best fit curve
% Input
%   ang      Angles used to simulate
%   eR_oppr  Error from Orth Procrustes
%   eR_fpro  Error from Full Procrustes
%   eR_mbpe  Error from Min Back Proj 
%   eR_epog  Error from Epipolar Geometry
%   i        Axis to plot x,y,z=1,2,3
%   axis     Axis Label
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
label = {'error oppr', 'error fpro', 'error mbpe', 'error epog'};

figure;
plot(ang, eR_oppr(i,:), 'bo');
hold on;
plot(ang, eR_fpro(i,:), 'gx');
hold on;
plot(ang, eR_mbpe(i,:), 'r+');
hold on;
plot(ang, eR_epog(i,:), 'ms');
hold on;
coeffs_oppr = polyfit(ang, eR_oppr(i,:), 1);
coeffs_fpro = polyfit(ang, eR_fpro(i,:), 1);
coeffs_mbpe = polyfit(ang, eR_mbpe(i,:), 1);
coeffs_epog = polyfit(ang, eR_epog(i,:), 1);
fittedX = linspace(min(ang), max(ang), 200);
fittedY_oppr = polyval(coeffs_oppr, fittedX);
fittedY_fpro = polyval(coeffs_fpro, fittedX);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
fittedY_epog = polyval(coeffs_epog, fittedX);
plot(fittedX, fittedY_oppr, 'b-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_fpro, 'g-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_epog, 'm-', 'LineWidth', 1);
title(axis);
legend(label,'Location','northeast');
xlabel('Degrees');
ylabel('Half Radians');

end