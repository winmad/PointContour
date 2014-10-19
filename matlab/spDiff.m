function res = spDiff(p , q)
    res = 0;
    len = size(p , 2);
    for i = 1 : len
        res = res + dist(p(:,i) , q(:,i));
    end
end

function d = dist(a , b)
    d = 0;
    for i = 1 : 3
        d = d + (a(i) - b(i)) * (a(i) - b(i));
    end
    d = sqrt(d);
end