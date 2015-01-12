function pts = resampleBsp(ctrlNodes , knots , t)
    bsp = struct('form' , 'B-' , 'number' , length(knots) - 4 , 'order' , 4 , 'dim' , 3 , ...
        'coefs' , ctrlNodes , 'knots' , knots);
    pts = fnval(bsp , t);
end