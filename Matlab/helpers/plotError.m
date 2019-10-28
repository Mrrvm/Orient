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
    nonzeros(j, :) = yvar(j,:) ~= 0;
end
for j=1:nMethods
    if sum(nonzeros(j, :))
        coeffs(j, :) = polyfit(var(nonzeros(j, :)), yvar(j, nonzeros(j, :)), 1);
        fittedX(j, :) = linspace(min(var(nonzeros(j, :))), max(var(nonzeros(j, :))), 200);
    end
end

for j=1:nMethods
    if sum(nonzeros(j, :))
        plot(var(nonzeros(j, :)), yvar(j, nonzeros(j, :)), strcat(colors{j},'.'), 'LineWidth', 1);
        hold on;
    end
end

for j=1:nMethods
    if sum(nonzeros(j, :))
        fittedY(j, :) = polyval(coeffs(j, :), fittedX(j, :));
        plot(fittedX(j, :), fittedY(j, :), strcat(colors{j},'-'), 'LineWidth', 1);
        hold on;
    end
end

title(pTitle);
legend(label,'Location','northeast');
xlabel(xLabel);
ylabel('Error (degrees)');

saveas(fig, strcat(saveDir, pTitle, '.fig'));
saveas(fig, strcat(saveDir, pTitle, '.png'));

end