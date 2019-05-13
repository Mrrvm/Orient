function filterdataMotive(imgDir, csvFilepath, savePath)
%filterdataMotive Filter data from images and csv file. It produces
% a structure images with all the images and a structure 
% rotations describing all the rotations with the respective
% indexes to address the imgs structure.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgDir        Images directory (.jpg or .png)
%   csvFilepath CSV motive file 
%   savePath     Directory where to save the new structures
% Output
%   images       Structure with all the images
%   rotations    Structure with  all the possible rotations
%                       contains the rotation vector (rot), 
%                       the index to the first and second image 
%                       (indImg1, inImg2) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imgs = dir(fullfile(imgDir, '*'));

csvFile = csvread(csvFilepath, 7, 1);
csvData = csvFile(:,1:10);
clear csvFile;

csvID = fopen(csvFilepath);
a = textscan(csvID,'%s','Delimiter','\n');
b = a{1,1};
firstLine = char(b(1,1));
clear a b;
splitLine = split(firstLine, ',');
startDate = splitLine(10);
[token remain] = strtok(startDate, ' ');
[token remain] = strtok(remain, ' ');
t = char(token);
startTime = datetime(t,'Format','HH.mm.ss.SSS');

j = 1;
for i=1:numel(imgs)   
    if(imgs(i).bytes ~= 0 && (~isempty(strfind(imgs(i).name, '.jpg')) || ~isempty(strfind(imgs(i).name, '.png'))))
         imgName = imgs(i).name;
         imgTstmp = datetime(imgName(1:(end-4)),'Format','HH.mm.ss.SSS');
         timePassed = duration(imgTstmp - startTime,'Format','hh:mm:ss.SSS');
         timePassedRep = repmat(timePassed, [size(csvData, 1), 1]);
         dataSeconds = seconds(csvData(:, 1));
         dataSeconds.Format =  'hh:mm:ss.SSS';
         [minValue,closestIndex] = min(abs(timePassedRep-dataSeconds));
         images(j).position = csvData(closestIndex, 7:9);
         img = fullfile(imgDir, imgName);
         images(j).img = rgb2gray(imread(img));
         j = j+1;
    end
end
clear csvData;

z = 1;
for i=1:(numel(images)-1)
    pos1 = images(i).position;
    for k=(i+1):numel(images)  
        pos2 = images(k).position;
        angle = acos(dot(pos1, pos2)/(norm(pos1)*norm(pos2)));
        vector = cross(pos1,pos2)/norm(cross(pos1,pos2));
        rotations(z).indImg1 = i;
        rotations(z).indImg2 = k;
        rotations(z).rot = vector*tan(angle/2);
        rotations(z).angle = angle;
        z = z + 1;
    end
end


mkdir(savePath);
save(strcat(savePath, 'images.mat') , 'images');
save(strcat(savePath, 'rotations.mat') , 'rotations');

end