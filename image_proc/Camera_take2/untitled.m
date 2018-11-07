n_points = 8;
x1 = 100*rand(n_points, 2);

K = [1 0 0; 0 1 0; 0 0 1]; % intrisics as identity for testing
fx = K(1,1); fy = K(2,2); 
cx = K(1,3); cy = K(2,3);

teta = pi/4;
R12 = [1 0 0; 0 cos(teta) -sin(teta); 0 sin(teta) cos(teta) ] % Rotation in x
T = [1-cos(teta) sin(teta) 0];

x1_homo = [x1 ones(n_points, 1)];
px1(:,1) = (x1_homo(:,1)-cx)/fx;
px1(:,2) = (x1_homo(:,2)-cy)/fy;
l1(:) = 1./sqrt(px1(:,1).^2 + px1(:,2).^2 + 1);
px1(:,3) = x1_homo(:, 3);

X = px1./l1';

px2 = (K*(R12*X' + T'))';
l2 = 1./px2(:, 3);
x2_homo = px2.*l2;



%% Normalization 

% Subtract the centroids
x1_centroid = [mean(x1_homo(:,1)) mean(x1_homo(:,2))];
x2_centroid = [mean(x2_homo(:,1)) mean(x2_homo(:,2))];

x1_translated = x1_homo(:,1:2) - x1_centroid;
x2_translated = x2_h(:,1:2) - x2_centroid;

% Isotropic scaling
x1_scale = (n_points*sqrt(2))/sum(sqrt((x1_translated(:,1).^2)+(x1_translated(:,2).^2)));
x2_scale = (n_points*sqrt(2))/sum(sqrt((x2_translated(:,1).^2)+(x2_translated(:,2).^2)));

T1 = [x1_scale 0 -x1_centroid(1)*x1_scale; 0 x1_scale -x1_centroid(2)*x1_scale; 0 0 1];
T2 = [x2_scale 0 -x2_centroid(1)*x2_scale; 0 x2_scale -x2_centroid(2)*x2_scale; 0 0 1];

x1_norm = (T1 * x1_homo')';
x2_norm = (T2 * x2_homo')';

%% Analytic 8 point algorithm 

U = [x2_norm(:,1).*x1_norm(:,1) x2_norm(:,1).*x1_norm(:,2) x2_norm(:,1) x2_norm(:,2).*x1_norm(:,1) x2_norm(:,2).*x1_norm(:,2) x2_norm(:,2) x1_norm(:,1) x1_norm(:,2) ones(n_points,1)];

%% Linear Least-Squares Technique (#1)

% Assume one f coefficients to be -1
% Expression to solve becomes ||Um * f'-c||^2
%   where Um is a [n_points 8] matrix and c is the column of Um to subtract
% The first derivative of the expression is 0, thus we can solve it easily
%   by doing, fm = inv(Um' * Um) * Um' * c
% Iterate over the 9 columns to know which one is better to subtract

fm = zeros(8, 9);
f = zeros(9, 9);

for i = 1:9
    Um = U;
    Um(:, i) = [];
    fm(:, i) = pinv(Um'*Um)*Um'*U(:, i);
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

[U, S, V] = svd(F);
Sc = S; Sc(3,3) = 0;

Fc = U*Sc*V';

%% Denormalization

finalF = T2'*Fc*T1;


%% Obtain R and t

E = K'*finalF*K;
[U, S, V] = svd(E);

W = [0 -1 0; 1 0 0; 0 0 1];

Rcalc1 = U * W' * V'
Rcalc2 = U * W * V'
Tcalc1 = U(:, 3)
Tcalc2 = -U(:, 3)

%% Procrustes 

[d, z, t] = procrustes(px1./l1', px2./l2');

t.T

