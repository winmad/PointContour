function [bsp , p] = convert2Spline(pts)
    N = size(pts , 2);
    curveLen = 0;
    for i = 1 : N - 1
        curveLen = curveLen + norm(pts(:,i) - pts(:,i+1));
    end
    L = max(2 , floor(curveLen * 3));
    bsp = spap2(L , 4 , 1:N , pts);
    
    M = size(bsp.coefs , 2);
    bsp.coefs(: , 1) = pts(: , 1);
    bsp.coefs(: , M) = pts(: , N);
    p = fnval(bsp , 1:N);
end
