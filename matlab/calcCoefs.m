function coefs = calcCoefs(bsp)
    n = bsp.knots(end);
    bs = bsp;
    k = size(bsp.coefs , 2);
    bs.coefs = eye(k);
    bs.dim = k;
    coefs = fnval(bs , 1:n);
    %pts = bsp.coefs * coefs;
end