function bestInds =  ransacByProcrustes(m1, m2, K, radius)
%ransacByProcrustes Ransac point matches by distance
%%%%%%%%%%%%%%%%%%%s%%%%%%%%%%%%%%%%%%%%%%
% Filter point matches by distance through ransac
% Input
%   m1,m2       2D points before and after
%                     transformation
%   nMatches   Number of point matches
% Output
%   l1, l2          2D points indexes to keep
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

maxIters = 20;
i = 0;
nRand = round(size(m1,2)*0.10);
maxErr =  0.0003;
enough = round(size(m1,2)*0.50);
bestScore = 0;

Ki = inv(K);
M1 = projectToSphere(double(m1), radius);
M1 = Ki*M1;
M2 = projectToSphere(double(m2), radius);
M2 = Ki*M2;

while i < maxIters
     firstInds = randi(size(M1,2), 1, nRand);
     maybeInliers1 = M1(:, firstInds);
     maybeInliers2 = M2(:, firstInds);
     R = findModel(maybeInliers1, maybeInliers2);
     aux = ones(1,size(M1,2));
     aux(firstInds) = 0;
     rest1 = M1(:, aux>0);
     rest2 = M2(:, aux>0);
     restInds =  testModel(rest1, rest2, maxErr, R);
     alsoInliers1 = M1(:, restInds>0);
     alsoInliers2 = M2(:, restInds>0);
     if size(alsoInliers1,2) > enough
         allInliers1 = [maybeInliers1 alsoInliers1];
         allInliers2 = [maybeInliers2 alsoInliers2];
         modelScore = sum(testModel(allInliers1, allInliers2, maxErr, R));
         if modelScore > bestScore
             bestR = R;
             bestScore = modelScore;
         end
     end
     i = i +1;
end

bestInds = testModel(M1, M2, maxErr, bestR);

end

function R = findModel(M1, M2)

A = M1*M2';
[U,S,V] = svd(A);
R = V*U';

end

function ind = testModel(M1, M2, maxErr, R)

    ind = zeros(1, size(M1,2));
    M2a = R*M1;
    err = sum(abs(M2-M2a));
    ind(err < maxErr) = 1;

end