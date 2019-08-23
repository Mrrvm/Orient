function filterdataGrid(imgDir, axis, savePath)
%filterdata Filter data from images. It produces
% a structure data with all the images and a structure 
% rotations describing all the rotations with the respective
% indexes to address the imgs structure.
% Images should all be in a directory and be named by the 
% following rule [#][+/-][angle in degrees]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgDir        Images directory (.jpg or .png)
%   axis            Axis being processed
%   savePath     Directory where to save the new structures
% Output
%   data            Structure with all the images
%   rotations    Structure with  all the possible rotations
%                       contains the rotation vector (rot), 
%                       the index to the first and second image 
%                       (indImg1, inImg2) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imgs = dir(fullfile(imgDir, '*'));
 
if axis == 'x'
    vector = [0 0 1];
 end
 if axis == 'y'
     vector = [0 1 0];
 end
 if axis == 'z'
     vector = [1 0 0];
 end
        
z = 1;
j = 1;
for i=1:numel(imgs)   
    if(imgs(i).bytes ~= 0 && (~isempty(strfind(imgs(i).name, '.jpg')) || ~isempty(strfind(imgs(i).name, '.png'))))
        imgName1= imgs(i).name;
        img1 = fullfile(imgDir, imgName1);
        data(j).img = rgb2gray(imread(img1));
        [token remain] = strtok(imgName1(2:end), '.');
        angle1 = str2double(token);
        j = j +1;
        for k=(i+1):numel(imgs)  
            if(imgs(k).bytes ~= 0 && (~isempty(strfind(imgs(k).name, '.jpg')) || ~isempty(strfind(imgs(k).name, '.png'))))
                 imgName2= data(k).name;
                 [token remain] = strtok(imgName2(2:end), '.');
                 angle2 = str2double(token);
                 angle = angle2-angle1;
                 rotations(z).indImg1 = j;
                 rotations(z).indImg2 = k;
                 rotations(z).rot = eul2rotm(angle*vector);
                 rotations(z).angle = angle;   
                 z = z + 1;
            end       
        end
    end
end

mkdir(savePath);
save(strcat(savePath, 'data.mat') , 'data');
save(strcat(savePath, 'rotations.mat') , 'rotations');

end