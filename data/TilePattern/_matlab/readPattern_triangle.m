mlStruct = parseXML('3^6_11100.xml');

triangleInfo = mlStruct.Children(2).Children.Children.Children;

ver = [];

tri = [];

for i = 1 : length(triangleInfo)
    currStr = triangleInfo(i).Attributes(1).Value;

    currData = sscanf(currStr, "%s %f %f %s %f %f %s %f %f");

    x1 = currData(2);   x2 = currData(5);   x3 = currData(8);
    y1 = currData(3);   y2 = currData(6);   y3 = currData(9);

    currVerNum = length(ver);

    v1_index = 0;
    v2_index = 0;
    v3_index = 0;

    flag1 = true;
    flag2 = true;
    flag3 = true;

    for j = 1 : currVerNum
        if (abs(ver(j, 1) - x1) < 0.1 && abs(ver(j, 2) - y1) < 0.1)
            v1_index = j;
            flag1 = false;

        elseif(abs(ver(j, 1) - x2) < 0.1 && abs(ver(j, 2) - y2) < 0.1)
            v2_index = j;
            flag2 = false;

        elseif(abs(ver(j, 1) - x3) < 0.1 && abs(ver(j, 2) - y3) < 0.1)
            v3_index = j ;
            flag3 = false;
        end
    end

    if (flag1) ver(end + 1, :) = [x1, y1, 0]; end
    if (flag2) ver(end + 1, :) = [x2, y2, 0]; end
    if (flag3) ver(end + 1, :) = [x3, y3, 0]; end

    if (v1_index == 0) v1_index = currVerNum + flag1; end
    if (v2_index == 0) v2_index = currVerNum + flag1 + flag2; end
    if (v3_index == 0) v3_index = currVerNum + flag1 + flag2 + flag3; end

    n = cross([x2 - x1, y2 - y1, 0], [x3 - x2, y3 - y2, 0]);

    if (sum(n) > 0)
        tri(i, :) = [v1_index, v2_index, v3_index];
    else
        tri(i, :) = [v3_index, v2_index, v1_index];
    end
end

ver = ver / 1000;

writeOBJ('3^6_11100.obj', ver,tri);