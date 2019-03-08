function r = matrix_to_axisangle(R)

rho = acos(0.5*(R(1,1)+R(2,2)+R(3,3)-1));

n(1) = 0.5*sin(rho)*(R(3,2)-R(2,3));
n(2) = 0.5*sin(rho)*(R(1,3)-R(3,1));
n(3) = 0.5*sin(rho)*(R(2,1)-R(1,2));

r = tan(rho*0.5)*n;

end