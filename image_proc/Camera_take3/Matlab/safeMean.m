function res = safeMean(val, option)

szh = size(val, 2);
szv = size(val,1);
    
if option == 2
    for j=1:szv
        res = zeros(szv, 1);
        count = 0;
        for i=1:szh
            if val(j, i) ~= 0
                res(j) = res(j) + val(j, i);
                count = count + 1;
            end
        end
        if res(j) ~= 0
            res(j) =  res(j)/count;
        end
    end
end

    
if option == 1
    for j=1:szh
        res = zeros(1, szh);
        count = 0;
        for i=1:szv
            if val(i, j) ~= 0
                res(j) = res(j) + val(i, j);
                count = count + 1;
            end
        end
        if res(j) ~= 0
         res(j) =  res(j)/count;
        end
    end
end


end

