function data = filterdata(imgDir, rotationsFile)

imgs = dir(fullfile(imgDir, '*'));

struct = load(rotationsFile);
fn = fieldnames(struct);
data = struct.(fn{1});
startDate = data(1,10);
[token remain] = strtok(startDate, ' ');
[token remain] = strtok(remain, ' ');
t = char(token);
startTime = datetime(t,'InputFormat','HH.mm.ss.SSS');

for i=1:numel(imgs)
    
    if(imgs(i).bytes ~= 0)
        imgName = imgs(i).name;
        imgTstmp = datetime(imgName,'InputFormat','HH.mm.ss.SSS');
    end
    
    
end

end