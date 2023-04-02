mlStruct = parseXML('6^3_18644.xml');

hexagonIndo = mlStruct.Children(2).Children.Children.Children;

hexa = [];
ver = [];

for i = 1 : length(hexagonIndo)
    currStr = hexagonIndo(i).Attributes(1).Value;

    currData = sscanf(currStr, "%s %f %f %s %f %f %s %f %f %s %f %f %s %f %f %s %f %f");

    x1 = currData(2);   x2 = currData(5);   x3 = currData(8);  x4 = currData(11);
    y1 = currData(3);   y2 = currData(6);   y3 = currData(9);  y4 = currData(12);

    x5 = currData(14);  x6 = currData(17);
    y5 = currData(15);  y6 = currData(18);

    currVerNum = length(ver);

    v1_index = 0;
    v2_index = 0;
    v3_index = 0;
    v4_index = 0;
    v5_index = 0;
    v6_index = 0;

    flag1 = true;
    flag2 = true;
    flag3 = true;
    flag4 = true;
    flag5 = true;
    flag6 = true;

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

        elseif(abs(ver(j, 1) - x4) < 0.1 && abs(ver(j, 2) - y4) < 0.1)
            v4_index = j ;
            flag4 = false;

        elseif(abs(ver(j, 1) - x5) < 0.1 && abs(ver(j, 2) - y5) < 0.1)
            v5_index = j ;
            flag5 = false;

        elseif(abs(ver(j, 1) - x6) < 0.1 && abs(ver(j, 2) - y6) < 0.1)
            v6_index = j ;
            flag6 = false;
        end
    end

    if (flag1) ver(end + 1, :) = [x1, y1, 0]; end
    if (flag2) ver(end + 1, :) = [x2, y2, 0]; end
    if (flag3) ver(end + 1, :) = [x3, y3, 0]; end
    if (flag4) ver(end + 1, :) = [x4, y4, 0]; end
    if (flag5) ver(end + 1, :) = [x5, y5, 0]; end
    if (flag6) ver(end + 1, :) = [x6, y6, 0]; end

    if (v1_index == 0) v1_index = currVerNum + flag1; end
    if (v2_index == 0) v2_index = currVerNum + flag1 + flag2; end
    if (v3_index == 0) v3_index = currVerNum + flag1 + flag2 + flag3; end
    if (v4_index == 0) v4_index = currVerNum + flag1 + flag2 + flag3 + flag4; end
    if (v5_index == 0) v5_index = currVerNum + flag1 + flag2 + flag3 + flag4 + flag5; end
    if (v6_index == 0) v6_index = currVerNum + flag1 + flag2 + flag3 + flag4 + flag5 + flag6; end

    n1 = cross([x2 - x1, y2 - y1, 0], [x3 - x2, y3 - y2, 0]);
    n2 = cross([x4 - x3, y4 - y3, 0], [x1 - x4, y1 - y4, 0]);

    n = (n1 + n2) / 2;

    if (sum(n) > 0)
        hexa(i, :) = [v1_index, v2_index, v3_index, v4_index, v5_index, v6_index];
    else
        hexa(i, :) = [v6_index, v5_index, v4_index, v3_index, v2_index, v1_index];
    end
end

ver = ver / 1000;

filename = '6^3_18644.obj';
f = fopen( filename, 'w' );
fprintf( f, ['v' repmat(' %0.13g',1,size(ver,2)) '\n'], ver');
fmt = repmat(' %d',1,size(hexa,2));
fprintf( f,['f' fmt '\n'], hexa');