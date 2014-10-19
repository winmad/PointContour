function curveNet = readCurves(filename)
    fp = fopen(filename , 'r');
    n = fscanf(fp , '%d' , 1);
    curveNet = struct('numLines' , n , 'len' , zeros(n , 1) , ...
        'lines' , zeros(n , 3 , 100));
    for i = 1 : n
        l = fscanf(fp , '%d' , 1);
        curveNet.len(i) = l;
        for j = 1 : l
            p = zeros(3 , 1);
            for k = 1 : 3
                p(k) = fscanf(fp , '%f ' , 1);
            end
            curveNet.lines(i , : , j) = p;
        end
    end
end