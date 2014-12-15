function [bsp , p] = convert2Spline(pts)
    N = size(pts , 2);
    t = zeros(1 , N);
    curveLen = 0;
    for i = 2 : N
        l = norm(pts(:,i) - pts(:,i-1));
        curveLen = curveLen + l;
        t(i) = curveLen;
    end
    t = t / curveLen;
    L = max(2 , floor(curveLen * 3));
    bsp = spap2(L , 4 , t , pts);
    M = size(bsp.coefs , 2);
    bsp.coefs(: , 1) = pts(: , 1);
    bsp.coefs(: , M) = pts(: , N);
    p = fnval(bsp , t);
end
