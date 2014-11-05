function pts = calcCoefs1(bsp , N)
    bs = fn2fm(bsp , 'pp-');
    pts = zeros(3 , N);
    l = size(bs.breaks , 2) - 1;
    j = 1;
    for i = 1 : N
        if (j < l) && (bs.breaks(j + 1) <= i)
            j = j + 1;
        end
        for k = 1 : 3
            pts(k , i) = polyval(bs.coefs((j - 1) * 3 + k , :) , i - bs.breaks(j));
        end
    end
end