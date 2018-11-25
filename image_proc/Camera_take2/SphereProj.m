function res = SphereProj(obj, radius, project)
%  Projects/Unprojects point in sphere
%
%  In:
%     obj:      Points to project/unproject
%     radius:   Radius of sphere
%     project:  Binary project/unproject
%  Out:
%      res:     Projected points

if project
    z = radius./(sqrt( 1 + (obj(:,1).^2) + (obj(:,2).^2) ));

    res(:,1) = z.*obj(:,1);
    res(:,2) = z.*obj(:,2);
    res(:,3) = z;
else
    z = obj(:,3);
    res(:,1) = obj(:,1)./z;
    res(:,2) = obj(:,2)./z;
end

end

