function filterdataMotive(imgDir, csvFilepath, savePath)
%filterdataMotive Filter data from images and a csv file. 
%
% It produces a structure data with all the images and a 
% structure rotations describing all the rotations with the 
% respective indexes to address the imgs structure.
%
% Images should all be in a directory and each image file
% should be named after its datetime (see 'converting.sh').
%
% The csv file is obtained using optitrack e.g.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgDir      Images directory (.jpg or .png)
%   csvFilepath CSV motive file 
%   savePath    Directory where to save the new structures
% Output
%   data        Structure with all the images and marker 
%                   positions
%   rotations   Structure with  all the possible rotations
%                   contains the rotation vector (rot), 
%                   the index to the first and second image 
%                   (indImg1, inImg2) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imgs = dir(fullfile(imgDir, '*'));

csvFile = csvread(csvFilepath, 7, 0);
csvData = [csvFile(:,2) csvFile(:,27:38)];
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
         if((sum(csvData(closestIndex, 2:4)) ~= 0) && (sum(csvData(closestIndex, 5:7)) ~= 0) ...
                 && (sum(csvData(closestIndex, 8:10)) ~= 0) && (sum(csvData(closestIndex, 11:13)) ~= 0) )
             data(j).marker(1,:) = csvData(closestIndex, 2:4);
             data(j).marker(2,:) = csvData(closestIndex, 5:7);
             data(j).marker(3,:) = csvData(closestIndex, 8:10);
             data(j).marker(4,:) = csvData(closestIndex, 11:13);       
             img = fullfile(imgDir, imgName);
             data(j).img = imread(img)';
             j = j+1;
         end
    end
end
clear csvData;

z = 1;
for i=1:(numel(data)-1)
    M1 = data(i).marker(:,:);
    for k=(i+1):numel(data)  
        M2 = data(k).marker(:,:);
        [d, Z, tr] = procrustes(M1, M2, 'reflection', false, 'scaling', false);
        a = rotm2eul(tr.T);
        [ang ind] = max(abs(a));
        if (ang*180/pi < 45)       
            rotations(z).angle = a(ind)*180/pi;
            rotations(z).indImg1 = i;
            rotations(z).indImg2 = k;
            rotations(z).rot = rotm2eul(tr.T);
            rotations(z).tr = tr.c(1, :);
            z = z + 1;
        end
    end
end

mkdir(savePath);
save(strcat(savePath, 'data.mat') , 'data');
save(strcat(savePath, 'rotations.mat') , 'rotations');

end