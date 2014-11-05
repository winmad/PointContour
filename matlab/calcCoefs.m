function coefs = calcCoefs(knots)
    n = knots(end);
    k = size(knots , 2) - 4;
    bsp = struct('form' , 'B-' , 'number' , k , 'order' , 4 , 'dim' , k , ...
        'knots' , knots , 'coefs' , eye(k));
    coefs = fnval(bsp , 1:n);
    %pts = ctrlNodes * coefs;
end