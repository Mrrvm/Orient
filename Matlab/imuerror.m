load('../input/camera/Ex2-Lab-Eye/rotations.mat');
gt = rotations;

load('../input/IMU_filter/filterdata/Ex2-Lab-Eye/imurotations.mat');
imu = imurotations;

n = 10;
z = 1;
c = 1;

for i=1:size(gt, 2)
    ind1gt = gt(i).indImg1;
    ind2gt = gt(i).indImg2;
    c = z;
    while c <= size(imu, 2)
        ind1imu = imu(c).ind1;
        ind2imu = imu(c).ind2;
        if(ind1imu == ind1gt && ind2imu == ind2gt)
            tt(1) = abs(gt(z).rot(1))-abs(imu(z).rot(1));
            tt(2) = abs(gt(z).rot(3))-abs(imu(z).rot(3));
            eR(z) = norm(tt);
            %angles(z) = gt(z).angle;
            axang = rotm2axang(eul2rotm(gt(z).rot));
            angles(z) = axang(4)*180/pi;
            z = z + 1;
            break;
        end
        c = c + 1;
    end
end
    

meR = mean(eR);
devrad = sqrt(sum((eR - meR).^2, 2)/size(gt, 2));
devdeg = sqrt(sum(((eR - meR)*180/pi).^2, 2)/size(gt, 2));


fprintf('Mean error is %f and variance is %f degrees\n', meR*180/pi, devdeg);
result = sprintf('\nMean: %f degrees \n Standard Deviation: %f degrees\n', meR*180/pi, devdeg);


fig = figure;
ptitle = 'Error per angle - IMU data';
saveDir = '../results/Matlab/real/imu/Ex2-Lab-Eye/';
hold on;
plot(angles, eR*180/pi, 'r.');
hold on;
coeffs = polyfit(angles, eR*180/pi, 1);
fittedX = linspace(min(angles), max(angles), 200);
fittedY = polyval(coeffs, fittedX);
plot(fittedX, fittedY, 'r-', 'LineWidth', 1);
title(ptitle);
%title(strcat(ptitle, result));
legend('errorIMU', 'Location','northeast');
xlabel('Angles (degrees)');
ylabel('Error (degrees)');

saveas(fig, strcat(saveDir, ptitle, '.fig'));
saveas(fig, strcat(saveDir, ptitle, '.png'));

