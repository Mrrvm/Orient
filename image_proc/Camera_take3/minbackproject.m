function [R, T] = minbackproject(m1, m2, B, N, R_pro)

I = [1 0 0; 0 1 0; 0 0 1];
Rinit = [R_pro(1,1:3) R_pro(2,1:3) R_pro(3,1:3)];
radius = 1;
%M1 = project_to_sphere(m1, radius);
%Zinit = M1(3,:);
Zinit =  ones(1,N);

options = optimset('MaxFunEvals',10000000);
[x,fval,exitflag,output] = fminsearch(@(x)objectivefun(x, m1, m2, B, N), [Rinit, Zinit], options);

R(1, 1:3) = x(1:3);
R(2, 1:3) = x(4:6);
R(3, 1:3) = x(7:9);
Z1 = x(10:end);
T = (R-I)*B;

end
