function entropy = findEntropy(sections)

sections_ = sections(:);
probs = sections(sections(:)>0);
entropy = 0;
sz = size(probs, 2);

for i=1:sz
    
    entropy = entropy + probs(i)*log2(probs(i));
    
end

entropy = - entropy;

end