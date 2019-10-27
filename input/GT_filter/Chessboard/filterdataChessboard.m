function filterdataChessboard(chessimgDir, imgDir, axis, savePath, cameraParams, squareSize, boardSize)
%filterdataChessboard Filter data from chessboard images. 
%
% It produces a structure data with all the images and a 
% structure rotations describing all the rotations with the 
% respective indexes to address the chessimgs structure.
%
% Images should all be in a directory
%
% e.g. arguments
% intrinsics = [1.1253e+03 0.945541684501329     9.960929796822086e+02;
%               0		   1.125630873153923e+03 7.543243830849593e+02;
%               0          0                     1                   ];
% tanDist = [4.7003e-04 -3.3083e-04];
% radialDist = [-0.3007 0.1288 -0.0318];
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

chessimgs = dir(fullfile(chessimgDir, '*'));
imgs = dir(fullfile(imgDir, '*'));
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

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
for i=1:numel(chessimgs)   
    if(chessimgs(i).bytes ~= 0 && (~isempty(strfind(chessimgs(i).name, '.jpg')) || ~isempty(strfind(chessimgs(i).name, '.png'))))
        
        chessimgName1 = chessimgs(i).name;
        chessimgFile1 = fullfile(chessimgDir, chessimgName1);
        
        imgName1 = imgs(i).name;
        imgFile1 = fullfile(imgDir, imgName1);
        
        data(j).img = imread(imgFile1);
        data(j).path = imgName1;
        
        chessdata = imread(chessimgFile1);
        
        [chessimgUn1, newOrigin] = undistortImage(chessdata, cameraParams,'OutputView','same');
        [chessimgPts1, boardSize] = detectCheckerboardPoints(chessimgUn1);
        chessimgPts1 = [chessimgPts1(:,1) + newOrigin(1), ...
             chessimgPts1(:,2) + newOrigin(2)];
        worldPoints = generateCheckerboardPoints(boardSize, squareSize);
        if (size(chessimgPts1, 1) == size(worldPoints, 1)) 
            [R1, t1] = extrinsics(chessimgPts1, worldPoints, cameraParams);
            c = j + 1;
            for k=(i+1):numel(chessimgs)  
                if(chessimgs(k).bytes ~= 0 && (~isempty(strfind(chessimgs(k).name, '.jpg')) || ~isempty(strfind(chessimgs(k).name, '.png'))))
                    
                    chessimgName2 = chessimgs(k).name;
                    chessimgFile2 = fullfile(chessimgDir, chessimgName2);
                    
                    [chessimgUn2, newOrigin] = undistortImage(imread(chessimgFile2), cameraParams,'OutputView','same');
                    [chessimgPts2, boardSize] = detectCheckerboardPoints(chessimgUn2);
                    chessimgPts2 = [chessimgPts2(:,1) + newOrigin(1), ...
                            chessimgPts2(:,2) + newOrigin(2)];
                    if (size(chessimgPts2, 1) == size(worldPoints, 1)) 
                        [R2, t2] = extrinsics(chessimgPts2, worldPoints, cameraParams);
                        R = R1'*R2;
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

