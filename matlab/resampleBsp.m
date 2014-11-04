function pts = resampleBsp(ctrlNodes , knots)
    bsp = struct('form' , 'B-' , 'number' , 5 , 'order' , 4 , 'dim' , 3 , ...
        'coefs' , ctrlNodes , 'knots' , knots);
    n = knots(end);
    pts = fnval(bsp , 1:n);
end