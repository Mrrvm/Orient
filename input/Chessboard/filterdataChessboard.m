function filterdataChessboard(imgDir, axis, savePath, cameraParams, squareSize, boardSize)
%filterdataChessboard Filter data from chessboard images. 
%
% It produces a structure data with all the images and a 
% structure rotations describing all the rotations with the 
% respective indexes to address the imgs structure.
%
% Images should all be in a directory
%
% e.g. arguments
% intrinsics = [1.1573e+03 -3.3579     975.9459; 
%               0          1.1584e+03  798.4888;
%               0          0           1      ];
% tanDist = [3.0406e-04 7.1815e-04];
% radialDist = [-0.3160 0.1699 -0.0569];
% squareSize = 21;
% boardSize = [8 10];
% cameraParams = cameraParameters('IntrinsicMatrix', intrinsics', ...
%   ...'RadialDistortion', radialDist, 'TangentialDistortion', tanDist); 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgDir       Images directory (.jpg or .png)
%   axis         Axis being processed ('x', 'y', 'z', or 'all')
%   cameraParams Parameters of the camera used 
%   savePath     Directory where to save the new structures
% Output
%   data         Structure with all the images
%   rotations    Structure with  all the possible rotations
%                   contains the rotation vector (rot), 
%                   the index to the first and second image 
%                   (indImg1, inImg2) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

worldPoints = generateCheckerboardPoints(boardSize, squareSize);
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
 if axis == 'all'
     vector = -1;
 end
        
z = 1;
j = 1;
c = 1;
for i=1:numel(imgs)   
    if(imgs(i).bytes ~= 0 && (~isempty(strfind(imgs(i).name, '.jpg')) || ~isempty(strfind(imgs(i).name, '.png'))))
        imgName1 = imgs(i).name;
        imgFile1 = fullfile(imgDir, imgName1);
        data(j).img = imread(imgFile1);
        data(j).path = imgName1;
        [imgUn1, newOrigin] = undistortImage(data(j).img, cameraParams,'OutputView','same');
        [imgPts1, boardSize] = detectCheckerboardPoints(imgUn1);
        if (size(imgPts1, 1) == size(worldPoints, 1)) 
            [R1, t1] = extrinsics(imgPts1, worldPoints, cameraParams);
            c = j + 1;
            for k=(i+1):numel(imgs)  
                if(imgs(k).bytes ~= 0 && (~isempty(strfind(imgs(k).name, '.jpg')) || ~isempty(strfind(imgs(k).name, '.png'))))
                    imgName2= imgs(k).name;
                    imgFile2 = fullfile(imgDir, imgName2);
                    [imgUn2, newOrigin] = undistortImage(imread(imgFile2), cameraParams,'OutputView','same');
                    [imgPts2, boardSize] = detectCheckerboardPoints(imgUn2);
                    if (size(imgPts2, 1) == size(worldPoints, 1)) 
                        [R2, t2] = extrinsics(imgPts2, worldPoints, cameraParams);
                        R = R2'*R1;
                        rotations(z).indImg1 = j;
                        rotations(z).indImg2 = c;
                        rotations(z).rot = rotm2eul(R);
                        if vector ~= -1
                            rotations(z).angle = vector*rotm2eul(R)'*180/pi;
                        end
                        z = z + 1;
                    end
                    c = c + 1;
                end       
            end
           
        end
        j = j + 1;
    end
end

mkdir(savePath);
save(strcat(savePath, '/data.mat') , 'data');
save(strcat(savePath, '/rotations.mat') , 'rotations');

end

