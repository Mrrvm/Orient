	function [ F_matrix ] = estimate_fundamental_matrix(Points_a,Points_b)

	s=size(Points_a);
	noPoints=s(1,1);

	transformed_Points_a=zeros(noPoints,3); transformed_Points_a(:,3)=1;
	transformed_Points_b=zeros(noPoints,3); transformed_Points_b(:,3)=1;


	%normalization

	%Step1: translation to center at origin
	meanX_a=0; meanX_b=0;
	meanY_a=0; meanY_b=0;

	for i=1:noPoints
	    meanX_a=meanX_a+Points_a(i,1);
	    meanY_a=meanY_a+Points_a(i,2);
	    meanX_b=meanX_b+Points_b(i,1);
	    meanY_b=meanY_b+Points_b(i,2);
	end
	meanX_a=meanX_a/noPoints; t_ax=-meanX_a;
	meanY_a=meanY_a/noPoints; t_ay=-meanY_a;
	meanX_b=meanX_b/noPoints; t_bx=-meanX_b;
	meanY_b=meanY_b/noPoints; t_by=-meanY_b;

	transformed_Points_a(:,1)=Points_a(:,1)-meanX_a;
	transformed_Points_a(:,2)=Points_a(:,2)-meanY_a;
	transformed_Points_b(:,1)=Points_b(:,1)-meanX_b;
	transformed_Points_b(:,2)=Points_b(:,2)-meanY_b;

	%Step2 isotropic scaling
	s_a=0; s_b=0;
	for i=1:noPoints
	    s_a=s_a+sqrt(transformed_Points_a(i,1)^2+transformed_Points_a(i,2)^2);
	    s_b=s_b+sqrt(transformed_Points_b(i,1)^2+transformed_Points_b(i,2)^2);
	end
	s_a=1/(noPoints*sqrt(2))*s_a;
	s_b=1/(noPoints*sqrt(2))*s_b;

	for i=1:noPoints
	    transformed_Points_a(i,1:2)=1/s_a*transformed_Points_a(i,1:2);
	    transformed_Points_b(i,1:2)=1/s_b*transformed_Points_b(i,1:2);
	end

	%Transformation matrix
	T_a=[1/s_a,0,1/s_a*t_ax;0,1/s_a,1/s_a*t_ay;0,0,1];
	T_b=[1/s_b,0,1/s_b*t_bx;0,1/s_b,1/s_b*t_by;0,0,1];

	%check mean and mean-distance of transformed points
	%scatter(transformed_Points_a(:,1),transformed_Points_a(:,2))
	%sum=0;
	%for i=1:noPoints
	%   sum=sum+norm(transformed_Points_a(i,1:2));
	%end
	%sum=1/noPoints*sum
	%mean(transformed_Points_b)

	A=zeros(noPoints,9);

	for i=1:noPoints
	   A(i,1)=transformed_Points_a(i,1)*transformed_Points_b(i,1);
	   A(i,2)=transformed_Points_a(i,1)*transformed_Points_b(i,2);
	   A(i,3)=transformed_Points_a(i,1);
	   A(i,4)=transformed_Points_a(i,2)*transformed_Points_b(i,1);
	   A(i,5)=transformed_Points_a(i,2)*transformed_Points_b(i,2);
	   A(i,6)=transformed_Points_a(i,2);
	   A(i,7)=transformed_Points_b(i,1);
	   A(i,8)=transformed_Points_b(i,2);
	   A(i,9)=1;
    end

	[U,S,V]=svd(A);
	F=V(:,end);
	F=reshape(F,[],3);
	[U,S,V]=svd(F);
	S(3,3)=0;
	F_scaled=U*S*V';

	F_scaled=F_scaled.*(1/F_scaled(3,3));
	F_matrix=transpose(T_b)*F_scaled*T_a;
	end