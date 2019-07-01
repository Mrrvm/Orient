function F = norm8point(m1, m2) 
%norm8point Apply norm8point algorithm to obtain
% the fundamental matrix F
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   m1,m2    2D points before and after
%                  transformation
% Output
%   R,T          Rotation and Translation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
nMatches = size(m1, 2);
m1h = [m1; ones(1, nMatches)];
m2h = [m2; ones(1, nMatches)];

[m1h, t1] = vision.internal.normalizePoints(m1h, 2, 'double');
[m2h, t2] = vision.internal.normalizePoints(m2h, 2, 'double');

f = zeros(nMatches, 9);
for idx = 1: nMatches
  f(idx,:) = [...
    m1h(1,idx)*m2h(1,idx), m1h(2,idx)*m2h(1,idx), m2h(1,idx), ...
    m1h(1,idx)*m2h(2,idx), m1h(2,idx)*m2h(2,idx), m2h(2,idx), ...
                 m1h(1,idx),              m1h(2,idx), 1];
end

[~, ~, vf] = svd(f, 0);
F = reshape(vf(:, end), 3, 3)';

[u, s, v] = svd(F);
s(end) = 0;
F = u * s * v';

F = t2' * F * t1;

F = F / norm(F);
if F(end) < 0
  F = -F;
end

end

