function showScenario(M1, M2, B, R, maxD)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show the camera positions and points
% Input
%   M1,M2   3D points before and after
%            transformation
%   B        Baseline
%   R        Rotation matrix
%   maxD     Maximum distance of the points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I = [1 0 0; 0 1 0; 0 0 1];
figure;
xlim([0 maxD]);
ylim([0 maxD]);
zlim([0 maxD]);
plot3(M1(1,:), M1(2,:), M1(3,:), 'o');
hold on;
plot3(M2(1,:), M2(2,:), M2(3,:), 'go')
hold on;
drawCuboid([0.03;0.03;0.03], B, I, [0.5 0.5 0.5], 0.5);
hold on;
drawCuboid([0.03;0.03;0.03], R*B, I, [0.5 0.9 0.4], 0.5);
hold on;
drawCuboid([0.02;0.02;0.02], [0;0;0], I, [0.5 0.5 0.5], 0.5);
hold on;
drawCuboid([0.02;0.02;0.02], [0;0;0], R, [0.5 0.9 0.4], 0.5);
line1 = [B'; [0,0,0]];
line2 = [(R*B)'; [0,0,0]];
plot3(line1(:,1)', line1(:,2)', line1(:,3)', 'k');
plot3(line2(:,1)', line2(:,2)', line2(:,3)', 'g');
end

