function [R, T] = minbackproject(m1, m2, B, N)

I = [1 0 0; 0 1 0; 0 0 1];
Rinit = [1 0 0 0    0.5253   -0.8509 0    0.8509    0.5253];
Zinit = ones(1, N);

options = optimset('MaxFunEvals',1000);
[x,fval,exitflag,output] = fminsearch(@(x)objectivefun(x, m1, m2, B, N), [Rinit, Zinit], options);

R(1, 1:3) = x(1:3);
R(2, 1:3) = x(4:6);
R(3, 1:3) = x(7:9);
Z1 = x(10:end);
T = (R-I)*B;

end
