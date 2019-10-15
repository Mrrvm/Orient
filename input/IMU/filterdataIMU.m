function filterdataIMU(dir, axis, savePath)


data = fopen(strcat(dir, 'imu.data'));

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
        
i = 1;
line = fgetl(data);
while ischar(line)
    split = textscan(line, '[%f, %f, %f, %f]');
    q(i, 1) = split{1};
    q(i, 2) = split{2};
    q(i, 3) = split{3};
    q(i, 4) = split{4};
    line = fgetl(data);
    i = i + 1;
end
 
q
%mkdir(savePath);
%save(strcat(savePath, '/rotations.mat') , 'rotations');

end

