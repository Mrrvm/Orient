function angles = generateAngles(nAngles, sigma)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate a normal distribution of angles
% Input
%   nAngles  Number of angles to generate
%   sigma    Normal d sigma
% Output
%   angles   Generated angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pd = makedist('Normal','mu',0,'sigma',sigma);

ang = zeros(1,nAngles);
for i=1:nAngles
    ang(i) = pd.random;
end

%figure;
%histfit(ang, nAngles,'Normal');

ang_rad = ang*pi/180;

angles(1      :nAngles,:)     = [ang_rad;          zeros(1,nAngles); zeros(1,nAngles)]';
angles((nAngles+1)  :2*nAngles,:)   = [zeros(1,nAngles); ang_rad;          zeros(1,nAngles)]';
angles((2*nAngles+1):3*nAngles,:)   = [zeros(1,nAngles); zeros(1,nAngles); ang_rad         ]';
        
end