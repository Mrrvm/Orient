function F = adjustFFrobeniusNorm(F)

    [u, s, v] = svd(F);
    s(end) = 0;
    F = u * s * v';

    F = F / norm(F);
    if F(end) < 0
      F = -F;
    end

end