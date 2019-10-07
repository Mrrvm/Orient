function [mn1, mn2, err, res] =  ransacByProcrustes(m1, m2, K, radius, minMatches, maxMatches, ransac)
%ransacByProcrustes Ransac point matches by distance
%%%%%%%%%%%%%%%%%%%s%%%%%%%%%%%%%%%%%%%%%%
% Filter point matches by distance through ransac
% Input
%   m1,m2       2D points before and after
%                     transformation
%   K                Intrinsics matrix
%   maxErr       Maximum error allowed for a good model
%   maxIters    Maximum number of iterations for ransac
% Output
%   l1, l2          2D points indexes to keep
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i = 0;
bestScore = 0;
err = 0;

M1 = projectToSphere(K, double(m1), radius);
M2 = projectToSphere(K, double(m2), radius);
maxIters = round(log(1-0.99)/(log(1-(1-ransac.outlierPer)^(minMatches)))); 

bestR = findModel(M1, M2);

while i < maxIters
     firstInds = randi(size(M1,2), 1, minMatches);
     maybeInliers1 = M1(:, firstInds);
     maybeInliers2 = M2(:, firstInds);
     R = findModel(maybeInliers1, maybeInliers2);
     aux = ones(1,size(M1,2));
     aux(firstInds) = 0;
     rest1 = M1(:, aux>0);
     rest2 = M2(:, aux>0);
     restInds =  testModel(rest1, rest2, ransac.maxErr, R);
     alsoInliers1 = M1(:, restInds>0);
     alsoInliers2 = M2(:, restInds>0);
     if size(alsoInliers1,2)+minMatches >= ransac.goodMatches
         allInliers1 = [maybeInliers1 alsoInliers1];
         allInliers2 = [maybeInliers2 alsoInliers2];
         modelScore = sum(testModel(allInliers1, allInliers2, ransac.maxErr, R));
         if modelScore > bestScore
             bestR = R;
             bestScore = modelScore;
         end
     end
     i = i +1;
end

bestInds = testModel(M1, M2, ransac.maxErr, bestR);

m1Best = m1(:, bestInds>0);
m2Best = m2(:, bestInds>0);
if sum(bestInds>0) <= ransac.goodMatches
    err = 1;
end
if sum(bestInds>0) >= maxMatches
    mn1 = double(m1Best(:, 1:maxMatches));
    mn2 = double(m2Best(:, 1:maxMatches));
else
    mn1 = double(m1Best(:, 1:sum(bestInds>0)));
    mn2 = double(m2Best(:, 1:sum(bestInds>0)));
end

res = 1- size(mn1,2)/size(m1, 2);

end


function R = findModel(M1, M2)

A = M1*M2';
[U,S,V] = svd(A);
R = V*U';

end

function ind = testModel(M1, M2, maxErr, R)

    ind = zeros(1, size(M1,2));
    M2a = R*M1;
    err = mean(M2-M2a);
    ind(abs(err) < maxErr) = 1;

end

