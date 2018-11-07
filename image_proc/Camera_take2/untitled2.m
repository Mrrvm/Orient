%//////////////////////////////////////////////////////////////////////////
%// Made by J.H.KIM, 2011 / feelmare@daum.net, feelmare@gmail.com        //
%// blog : http://feelmare.blogspot.com                                  //
%// Eight-Point Algorithm
%//////////////////////////////////////////////////////////////////////////

clc; clear all; close all;

% Corresponding points between two images
% sample #1 I11.jpg, I22.jpg
%{
load I11.txt; load I22.txt;
m1 = I11; m2 = I22;
%}

n_points = 8;
x1 = 100*rand(n_points, 2);

K = [1 0 0; 0 1 0; 0 0 1]; % intrisics as identity for testing
teta = pi;

R12 = [cos(teta) -sin(teta) 0; sin(teta) cos(teta) 0; 0 0 1]
T = [10 20 0];

x1_homo = [x1 ones(n_points, 1)];
X1 = K\x1_homo';
X2 = R12*X1+T';
x2_homo = (K*X2)';
x2 = x2_homo(:,1:2);

m1 = x1;
m2 = x2;

s = length(m1);
m1=[m1(:,1) m1(:,2) ones(s,1)];
m2=[m2(:,1) m2(:,2) ones(s,1)];
Width = 800; %image width
Height = 600; %image height

K = [1 0 0; 0 1 0; 0 0 1];

%% Normalization 

% Subtract the centroids
x1_centroid = [mean(x1(:,1)) mean(x1(:,2))];
x2_centroid = [mean(x2(:,1)) mean(x2(:,2))];

x1_translated = x1 - x1_centroid;
x2_translated = x2 - x2_centroid;

% Isotropic scaling
x1_scale = (n_points*sqrt(2))/sum(sqrt((x1_translated(:,1).^2)+(x1_translated(:,2).^2)));
x2_scale = (n_points*sqrt(2))/sum(sqrt((x2_translated(:,1).^2)+(x2_translated(:,2).^2)));

T1 = [x1_scale 0 -x1_centroid(1)*x1_scale; 0 x1_scale -x1_centroid(2)*x1_scale; 0 0 1];
T2 = [x2_scale 0 -x2_centroid(1)*x2_scale; 0 x2_scale -x2_centroid(2)*x2_scale; 0 0 1];

x1_norm = (T1 * x1_homo')';
x2_norm = (T2 * x2_homo')';

% Af=0 
A=[x1_norm(:,1).*x2_norm(:,1) x1_norm(:,2).*x2_norm(:,1) x2_norm(:,1) x1_norm(:,1).*x2_norm(:,2) x1_norm(:,2).*x2_norm(:,2) x2_norm(:,2) x1_norm(:,1) x1_norm(:,2), ones(s,1)];

% Get F matrix
[U D V] = svd(A);
F=reshape(V(:,9), 3, 3)';
% make rank 2 
[U D V] = svd(F);
F=U*diag([D(1,1) D(2,2) 0])*V';

% Denormalize
F = T2'*F*T1;
%Verification
%L1=F*m1'; m2(1,:)*L1(:,1); m2(2,:)*L1(:,2); m2(3,:)*L1(:,3);

%%
%Get E
E=K'*F*K;
% Multiple View Geometry 259page
%Get 4 Possible P matrix 
P4 = get4possibleP(E);
%Get Correct P matrix 
inX = [m1(1,:)' m2(1,:)'];
P1 = [eye(3) zeros(3,1)];
P2 = getCorrectCameraMatrix(P4, K, K, inX)

%%
%Get 3D Data using Direct Linear Transform(Linear Triangular method)
Xw = Triangulation(m1',K*P1, m2',K*P2);
xx=Xw(1,:);
yy=Xw(2,:);
zz=Xw(3,:);

figure(1);
plot3(xx, yy, zz, 'r+');

function P4 = get4possibleP(E)

% SVD of E
[U,S,V] = svd(E);

%W
W = [0,-1,0;1,0,0;0,0,1];        

P4 = zeros(3,4,4);
R1 = U*W*V';

R2 = U*W'*V';
T1 = U(:,3);
T2 = -U(:,3);

P4(:,:,1) = [R1, T1];
P4(:,:,2) = [R1, T2];
P4(:,:,3) = [R2, T1];
P4(:,:,4) = [R2, T2];
  
end

% getCorrectCameraMatrix - Check which of the 4 P solutions from the
%                          essential matrix is the correct one
%
%
% Given the essential matrix, two matching points in two images, and the
% camera calibration of both images, it checks which of the 4 possible
% solutions (p259) is the correct one by reprojecting in 3D.
%
%
% Input  - E   -> 3x3 essential matrix
%        - K1  -> 3x3 Camera calibration of image 1
%        - K2  -> 3x3 Camera calibration of image 2
%        - X   -> 3x2 homogeneous points in images 1 and 2
%
% Output - P   -> 3x4 Correct camera matrix (rotation and translation)
%
%
%
% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% iesteban@science.uva.nl
% isaac.esteban@tno.nl

function [P] = getCorrectCameraMatrix(PXcam, K1,K2, X)

    % Two matching points in image coordinates (x in image 1 and xp in
    % image 2)
    x = X(:,1);
    xp = X(:,2);
  
    % The first camera matrix is taken P = [I|0] and the other 
    Pcam = [eye(3),zeros(3,1)];
    P = K1*Pcam;
    xhat = inv(K1)*x;
      
    
    % For each camera matrix (Pxcam), reproject the pair of points in 3D
    % and determine the depth in 3D of the point
    % FIRST I DO IT FOR ONE
    X3D = zeros(4,4);
    Depth = zeros(4,2);
    for i=1:4
        
        % First the point is converted
        xphat = inv(K2)*xp;

        % We build the matrix A
        A = [Pcam(3,:).*xhat(1,1)-Pcam(1,:);
             Pcam(3,:).*xhat(2,1)-Pcam(2,:);
             PXcam(3,:,i).*xphat(1,1)-PXcam(1,:,i);
             PXcam(3,:,i).*xphat(2,1)-PXcam(2,:,i)];
        
        % Normalize A
        A1n = sqrt(sum(A(1,:).*A(1,:)));
        A2n = sqrt(sum(A(2,:).*A(2,:)));
        A3n = sqrt(sum(A(1,:).*A(1,:)));
        A4n = sqrt(sum(A(1,:).*A(1,:))); 
        Anorm = [A(1,:)/A1n;
                 A(2,:)/A2n;
                 A(3,:)/A3n;
                 A(4,:)/A4n];
             
        % Obtain the 3D point
        [Uan,San,Van] = svd(Anorm);
        X3D(:,i) = Van(:,end);
        
        % Check depth on second camera
        xi = PXcam(:,:,i)*X3D(:,i);
        w = xi(3);
        T = X3D(end,i);
        m3n = sqrt(sum(PXcam(3,1:3,i).*PXcam(3,1:3,i)));
        Depth(i,1) = (sign(det(PXcam(:,1:3,i)))*w)/(T*m3n);
        
        % Check depth on first camera
        xi = Pcam(:,:)*X3D(:,i);
        w = xi(3);
        T = X3D(end,i);
        m3n = sqrt(sum(Pcam(3,1:3).*Pcam(3,1:3)));
        Depth(i,2) = (sign(det(Pcam(:,1:3)))*w)/(T*m3n);
         
    end;

    % Check which solution is the right one and return
    if(Depth(1,1)>0 && Depth(1,2)>0)
        P = PXcam(:,:,1);
    elseif(Depth(2,1)>0 && Depth(2,2)>0)
        P = PXcam(:,:,2);    
    elseif(Depth(3,1)>0 && Depth(3,2)>0)
        P = PXcam(:,:,3);
    elseif(Depth(4,1)>0 && Depth(4,2)>0)
        P = PXcam(:,:,4);
    end
end


function Xw = Triangulation(x1,P1,x2,P2)

% Argument
%   x1,x2 = Point in 2D space [x1,...,xn ; y1,...,yn ; 1,...,1]
%   P1,P2 = Projection Transformation Matrix
%   F = Fundamental Matrix
%   Xw = Point in 3D space [X1,...,Xn ; Y1,...,Yn ; Z1,...,Zn ; 1,...,1]

x11 = x1; %- [K1(1,3)*ones(1,size(x1,2)); K1(2,3)*ones(1,size(x1,2)) ; zeros(1,size(x1,2))];
x22 = x2; %- [K2(1,3)*ones(1,size(x2,2)); K2(2,3)*ones(1,size(x2,2)) ; zeros(1,size(x2,2))];


for i=1:size(x11,2)
    %Select point
    sx1 = x11(:,i);
    sx2 = x22(:,i);
    
    
    A1 = sx1(1,1).*P1(3,:) - P1(1,:);
    A2 = sx1(2,1).*P1(3,:) - P1(2,:);
    A3 = sx2(1,1).*P2(3,:) - P2(1,:);
    A4 = sx2(2,1).*P2(3,:) - P2(2,:);
    
    %Set A matric, and find SDV(A)
    A = [A1;A2;A3;A4];
    [U,D,V] = svd(A);
    
    %Point in 3D space is the last column of V
    X_temp = V(:,4);
    X_temp = X_temp ./ repmat(X_temp(4,1),4,1);
    
    Xw(:,i) = X_temp;
    
end
end