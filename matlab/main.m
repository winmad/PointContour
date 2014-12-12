curveNet = readCurves('curves.txt');
figure;
hold on;
N = 1;
tot = 0;
numPoints = 0;
for i = 1 : N
    fprintf('-------- Line %d -----------\n' , i);
    len = curveNet.len(i);
    x = curveNet.lines(i , 1 , 1:len);
    y = curveNet.lines(i , 2 , 1:len);
    z = curveNet.lines(i , 3 , 1:len);
    x = reshape(x , [1 , len]);
    y = reshape(y , [1 , len]);
    z = reshape(z , [1 , len]);
    %plot3(x , y , z , 'b' , 'LineStyle' , '-' , 'LineWidth' , 2);
    %pts = [x(2:len-1); y(2:len-1); z(2:len-1)];
    pts = [x(1:len); y(1:len); z(1:len)];
    [bs , val] = convert2Spline(pts);
    %[bs , val] = convert2Nurbs(pts);
    fnplt(bs , 'r');
    polyBs = bs.coefs;
    plot3(polyBs(1,:) , polyBs(2,:) , polyBs(3,:) , 'b');
%     cx = [x(1) , val(1,1)];
%     cy = [y(1) , val(2,1)];
%     cz = [z(1) , val(3,1)];
%     plot3(cx, cy, cz, 'b' , 'LineWidth' , 2);
%     cx = [val(1,len-2) , x(len)];
%     cy = [val(2,len-2) , y(len)];
%     cz = [val(3,len-2) , z(len)];
%     plot3(cx, cy, cz, 'b' , 'LineWidth' , 2);
    tot = tot + size(bs.coefs , 2);
    numPoints = numPoints + len;
end
fprintf('%d / %d\n' , tot , numPoints);