function hist = createHistogram(eR, bins)

sz = size(eR, 2);
hist = zeros(1, 6);

for i=1:sz
    if(eR(i) < bins(1))
        hist(1) = hist(1) + 1;
    else
        if(eR(i) < bins(2))
            hist(2) = hist(2) + 1;
        else
            if(eR(i) < bins(3))
                hist(3) = hist(3) + 1;
            else
                if(eR(i) < bins(4))
                    hist(4) = hist(4) + 1;
                    else
                    if(eR(i) < bins(5))
                        hist(5) = hist(5) + 1;
                    else
                        hist(6) = hist(6) + 1;
                    end
                end
            end
        end
    end
end

end