function sections = whichImageSections(m, imageSize) 

sz = size(m, 2);

imszX1 = round(imageSize(1)/3);
imszX2 = round(2*imageSize(1)/3);

imszY1 = round(imageSize(2)/3);
imszY2 = round(2*imageSize(2)/3);

sections = zeros(3,3);
sectionsPercentage = zeros(3,3);

for i=1:sz
    
    x = m(1, i);
    y = m(2, i);
    
    if x < imszX1
        col = 1;
    else
        if x < imszX2
            col = 2;
        else
            col = 3;
        end
    end
    
    if y < imszY1
        row = 1;
    else
        if y < imszY2
            row = 2;
        else
            row = 3;
        end
    end
    
    sections(row, col) = sections(row, col) + 1;
    
end

sections = sections/sz;


end