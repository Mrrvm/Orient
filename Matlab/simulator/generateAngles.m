function angles = generateAngles(nAngles, sigma)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate a normal distribution of angles
% Input
%   nAngles  Number of angles to generate
%   sigma    Normal d sigma
% Output
%   angles   Generated angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%pd = makedist('Uniform','Lower', -sigma, 'Upper', sigma);
pd = makedist('normal', 0, sigma);

ang = zeros(1,nAngles);
for i=1:nAngles
    ang(i) = pd.random;
end

figure;
histfit(ang, nAngles,'Normal');

angles = ang*pi/180;

        
end