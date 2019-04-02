function [R, T] = epipolargeo(m1, m2, K)

% Estimates fundamental matrix using LMedS
F = estimateFundamentalMatrix(m1', m2');

% Obtain essential matrix
E = K'*F*K;

% Obtain R 
W = [0 -1 0; 1 0 0; 0 0 1];
[U,S,V] = svd(E);
R1 = U*W'*V';
R2 = U*W*V';

% Obtain T
T1 = U(:,3);
T2 = -T1;

% Test all solutions
p1 = [m1(:,1); 1];
p2 = [m2(:,1); 1];

p2l = R1*p1 + T1; 
err(1) = norm(p2 - p2l);

p2l = R1*p1 + T2; 
err(2) = norm(p2 - p2l);

p2l = R2*p1 + T1; 
err(3) = norm(p2 - p2l);

p2l = R2*p1 + T2; 
err(4) = norm(p2 - p2l);

[a chosen] = min(err);

if chosen == 1
    R = R1; T = T1;
else
    if chosen == 2
        R = R1; T = T2;
    else
        if chosen == 3
            R = R2; T = T1;
        else
            R = R2; T = T2;
        end
    end
end
    
end
