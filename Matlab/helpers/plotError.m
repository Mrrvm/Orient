function plotError(var, yvar, xLabel, pTitle, saveDir, methods, colors)
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
nMethods = size(methods, 2);
label = strcat('error ', methods);

fig = figure;
for j=1:nMethods
    coeffs(j, :) = polyfit(var, yvar(j,:), 1);
end
fittedX = linspace(min(var), max(var), 200);
for j=1:nMethods
    fittedY(j, :) =  polyval(coeffs(j, :), fittedX);
    plot(fittedX, fittedY(j, :), strcat(colors{j},'-'), 'LineWidth', 1);
    hold on;
end
title(pTitle);
legend(label,'Location','northeast');
xlabel(xLabel);
ylabel('Error (degrees)');
saveas(fig, strcat(saveDir, pTitle, '.fig'));
saveas(fig, strcat(saveDir, pTitle, '.png'));

end