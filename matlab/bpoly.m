function ply = bpoly(t , knots)
    ply = 0;
    for i = 1 : 5
        term = 1;
        k = 4;
        for j = 1 : 5
            if (abs(knots(i) - knots(j)) < 1e-6)
                k = k - 1;
                continue;
            end
            term = term / (knots(i) - knots(j));
        end
        term = term * (t - knots(i)) ^ k;
        ply = ply + term;
    end
end