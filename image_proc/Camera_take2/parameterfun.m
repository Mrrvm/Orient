function min=parameterfun(x, p1, p2, baseline, K)
    
    size = length(p1);
    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,2);
    cy = K(2,3);
    R = [x(1) x(2) x(3); x(4) x(5) x(6); x(7) x(8) x(9)];
    z = x(10:end);

    P1 = [(z.*(p1(:,1)-cx)/fx) (z.*(p1(:,2)-cy)/fy) z]';

    P2 = [(z.*(p2(:,1)-cx)/fx) (z.*(p2(:,2)-cy)/fy) z]';

    t = (R*baseline'-baseline');

    diff = P2' - (R*P1 + t)';
    m = 0;
    for i=1:size
        m = m + norm(diff(i,:),2);
    end
    min = m;

  
end