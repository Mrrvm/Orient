function plotError(var, yvar, xLabel, pTitle, saveDir)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot error in terms of distances or 
% or noises with a  best fit curve
% Input
%   var             Distance to the camera or 
%                        pixel noise
%   xLabel        x label
%   pTitle         Plot title
%   eR_AllAxis Mean error of all axis 
%                        per method
%   saveDir      Directory to save graphs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
label = {'error oppr', 'error mee', 'error mbpe', 'error n8p'};

fig = figure;
% plot(var, yvar(1,:), 'bo');
% hold on;
% plot(var, yvar(2,:), 'gx');
% hold on;
% plot(var, yvar(3,:), 'r+');
% hold on;
% plot(var, yvar(4,:), 'ms');
% hold on;
coeffs_oppr = polyfit(var, yvar(1,:), 1);
coeffs_mee = polyfit(var, yvar(2,:), 1);
coeffs_mbpe = polyfit(var, yvar(3,:), 1);
coeffs_n8p = polyfit(var, yvar(4,:), 1);
fittedX = linspace(min(var), max(var), 200);
fittedY_oppr = polyval(coeffs_oppr, fittedX);
fittedY_mee = polyval(coeffs_mee, fittedX);
fittedY_mbpe = polyval(coeffs_mbpe, fittedX);
fittedY_n8p = polyval(coeffs_n8p, fittedX);
plot(fittedX, fittedY_oppr, 'b-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_mee, 'g-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_mbpe, 'r-', 'LineWidth', 1);
hold on;
plot(fittedX, fittedY_n8p, 'm-', 'LineWidth', 1);
title(pTitle);
legend(label,'Location','northeast');
xlabel(xLabel);
ylabel('Error (degrees)');
saveas(fig, strcat(saveDir, pTitle, '.fig'));
saveas(fig, strcat(saveDir, pTitle, '.png'));

end