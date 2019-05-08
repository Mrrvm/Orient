function newData = filterdata(imgDir, rotationsFile)

imgs = dir(fullfile(imgDir, '*'));

clear struct;
struct = load(rotationsFile);
fn = fieldnames(struct);
data = struct.(fn{1});
startDate = data(1,10);
[token remain] = strtok(startDate, ' ');
[token remain] = strtok(remain, ' ');
t = char(token);
startTime = datetime(t,'Format','HH.mm.ss.SSS');

j = 1;

for i=1:numel(imgs)   
    if(imgs(i).bytes ~= 0)
         imgName = imgs(i).name;
         imgTstmp = datetime(imgName(1:(end-4)),'Format','HH.mm.ss.SSS');
         timePassed = duration(imgTstmp - startTime,'Format','hh:mm:ss.SSS');
         aux = repmat(timePassed, [size(data, 1)-7+1, 1]);
         s = seconds(str2double(data(7:end, 2)));
         s.Format =  'hh:mm:ss.SSS';
         [minValue,closestIndex] = min(abs(aux-s));
         newData(j).position = data(closestIndex, 7:9);
         img = fullfile(imgDir, imgName);
         grayimg = rgb2gray(imread(img));
         newData(j).img = grayimg;
         j = j+1;
    end
end

save(strcat(string(fn), '.mat'), 'newData');

end