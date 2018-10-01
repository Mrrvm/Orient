n_points = 8;
x1 = 100*rand(n_points, 2);
x2 = 100*rand(n_points, 2);

x1_homo = [x1 ones(n_points, 1)];
x2_homo = [x2 ones(n_points, 1)];

%% Normalization 

% Subtract the centroids
x1_centroid = [mean(x1(:,1)) mean(x1(:,2))];
x2_centroid = [mean(x2(:,1)) mean(x2(:,2))];

x1_translated = x1 - x1_centroid;
x2_translated = x2 - x2_centroid;

% Isotropic scaling
x1_scale = (n_points/sqrt(2))/sum(sqrt((x1_translated(:,1).^2)+(x1_translated(:,2).^2)));
x2_scale = (n_points/sqrt(2))/sum(sqrt((x2_translated(:,1).^2)+(x2_translated(:,2).^2)));

T1 = [x1_scale 0 -x1_centroid(1)*x1_scale; 0 x1_scale -x1_centroid(2)*x1_scale; 0 0 1];
T2 = [x2_scale 0 -x2_centroid(1)*x2_scale; 0 x2_scale -x2_centroid(2)*x2_scale; 0 0 1];

x1_norm = (T1 * x1_homo')';
x2_norm = (T2 * x2_homo')';

%% Analytic 8 point algorithm 

U = [x2_norm(:,1).*x1_norm(:,1) x2_norm(:,1).*x1_norm(:,2) x2_norm(:,1) ...
    x2_norm(:,2).*x1_norm(:,1) x2_norm(:,2).*x1_norm(:,2) x2_norm(:,2) x1_norm(:,1) x1_norm(:,2) ones(n_points,1)];

%% Linear Least-Squares Technique (#1)

% Assume one f coefficients to be -1
% Expression to solve becomes ||Um * f'-c||^2
%   where Um is a [n_points 8] matrix and c is the column of Um to subtract
% The first derivative of the expression is 0, thus we can solve it easily
%   by doing, f = inv(Um' * Um) * Um' * c
% Iterate over the 9 columns to know which one is better to subtract

fm = zeros(8, 9);
f = zeros(9, 9);

for i = 1:9
    Um = U;
    Um(:, i) = [];
    fm(:, i) = (Um'*Um)\(Um'*U(:, i));
    if i == 1
        f(:, 1) = [-1 fm(:, 1)'];
    elseif i == 9
        f(:, 9) = [fm(:, 9)' -1];
    else
        f(:, i) = [fm(1:(i-1), i)' -1 fm(i:8, i)'];
    end
end


[h,best_col] = min(abs(mean(U*f)));
F = [f(1:3, best_col)'; f(4:6, best_col)'; f(7:9, best_col)'];
 
%% Constraint enforcement

% Replace the new found F by Fc, such that det(Fc) = 0 using SVD
% We are imposing the rank-2 constraint, which helps at reducing noise
% This is called minimizing the frobenius norm (F-Fc)
% F = USV' and Fc = UScV'

[U,S,V] = svd(F);

Sc = S; Sc(3,3) = 0;

Fc = U*Sc*V';

%% Denormalization

realF = T2'*Fc*T1;

