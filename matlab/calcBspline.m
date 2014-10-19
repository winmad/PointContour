function [bs , val] = calcBspline(pts , err)
    len = size(pts , 2);
%     bs = spaps(1:len , pts , err);
%     numCoefs = size(bs.coefs , 2);
%     bs.coefs(:,1) = pts(:,1);
%     bs.coefs(:,numCoefs) = pts(:,len);

    for nodes = 2 : 2
        bs = spap2(nodes , 4 , 1:len , pts);
        numCoefs = size(bs.coefs , 2);
        bs.coefs(:,1) = pts(:,1);
        bs.coefs(:,numCoefs) = pts(:,len);
        
        diff = spDiff(fnval(bs , 1:len) , pts);
        %fprintf('nodes = %d, diff = %.6f\n' , nodes , diff);
        if (diff < err)
            break;
        end
    end
    val = fnval(bs , 1:len);
end

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