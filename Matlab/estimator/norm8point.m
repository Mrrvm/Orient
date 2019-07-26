function F = norm8point(m1, m2, N1, N2) 
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

m1n = N1*m1h;
m2n = N2*m2h;

f = zeros(nMatches, 9);
for idx = 1: nMatches
  f(idx,:) = [ m1n(1,idx)*m2n(1,idx), m1n(2,idx)*m2n(1,idx), m2n(1,idx), ...
                   m1n(1,idx)*m2n(2,idx), m1n(2,idx)*m2n(2,idx), m2n(2,idx), ...
                   m1n(1,idx), m1n(2,idx), 1];
end

[~, ~, vf] = svd(f, 0);
F = reshape(vf(:, end), 3, 3)';

end

