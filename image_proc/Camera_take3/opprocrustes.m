function [R, T] = opprocrustes(m1, m2)

radius = 1;
M1 = project_to_sphere(m1, radius);
M2 = project_to_sphere(m2, radius);

[d,Z,tr] = procrustes(M1', M2', 'scaling', false, 'reflection', false); 
T = tr.c(1,:);
R = tr.T;


end
