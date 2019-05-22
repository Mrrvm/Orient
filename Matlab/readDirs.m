function [dirs, dists] = readDirs(allFilesDir)
%readDirs Read directories representing each distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input
%   imgDir       Image directory to read from
% Output
%   dirs            Available directories
%   dists          Corresponding distances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dirs = dir(fullfile(allFilesDir,'d*'));

for k = 1:numel(dirs)
    dists(k) = str2double(dirs(k).name(2:end));
end