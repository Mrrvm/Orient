function res = SphereProj(obj, radius, project, K)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Projects/Unprojects point in sphere
%
%  In:
%     obj:       Points to project/unproject
%     radius:    Radius of sphere
%     project:   Binary project/unproject
%     K: 3x3 matrix with camera's intrinsic parameters
%  Out:
%      res:      Projected points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,2);
    cy = K(2,3);
    s = K(1,2); 

    if project
        u = (obj(:,1)-cx)/fx;
        v = (obj(:,2)-cy)/fy;
        z = radius./(sqrt( 1 + (u.^2) + (v.^2) ));

        res(:,1) = z.*u;
        res(:,2) = z.*v;
        res(:,3) = z;
    else
        z = obj(:,3);
        res(:,1) = fx*(obj(:,1)./z)+cx;
        res(:,2) = fy*(obj(:,2)./z)+cy;
    end

end

