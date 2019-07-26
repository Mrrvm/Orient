function N = getNormalizationMatrix(m) 

    mu = mean(m, 2);
    mc = m - repmat(mu, 1, size(m, 2));
    norm = sqrt(sum(sum(mc.^2, 1)));
    if((1/norm ~= Inf))
        N = [sqrt(2)/norm 0 -mu(1)*(sqrt(2)/norm); 0 1 -mu(2)*(sqrt(2)/norm); 0 0 1];   
    else
        N = [1 0 0; 0 1 0; 0 0 1];
end