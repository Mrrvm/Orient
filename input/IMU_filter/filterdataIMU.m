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
 
z = 1;
sizez = size(q,1);
for i=1:sizez
    quat1 = q(i, :);
    for j=(i+1):sizez
        quat2 = q(j, :);
        eul = Quat2Eul(QuatMultiply(quat2, QuatConjugate(quat1)));
        imurotations(z).ind1 = i;
        imurotations(z).ind2 = j;
        % careful with coordinate systems
        imurotations(z).rot = [-eul(3) -eul(1) -eul(2)];
        if vector ~= -1
            imurotations(z).angle = vector*imurotations(z).rot'*180/pi;
        end
        z = z+1;
    end
end
        
mkdir(savePath);
save(strcat(savePath, '/imurotations.mat') , 'imurotations');

end


function quatc = QuatConjugate(quat) 

    quatc(1) = quat(1);
    quatc(2) = -quat(2);
    quatc(3) = -quat(3);
    quatc(4) = -quat(4);

end

function quatm = QuatMultiply(quat1, quat2)
    
    a1 = quat1(1);
    b1 = quat1(2);
    c1 = quat1(3);
    d1 = quat1(4);

    a2 = quat2(1);
    b2 = quat2(2);
    c2 = quat2(3);
    d2 = quat2(4);
    
    quatm(1) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    quatm(2) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    quatm(3) = a1*c2 - b1*d2 + c1*a2 + d1*b2;
    quatm(4) = a1*d2 + b1*c2 - c1*b2 + d1*a2;

end

function eul = Quat2Eul(q) 

    sinr_cosp = 2 * (q(1)*q(2) + q(3)*q(4));
    cosr_cosp = 1 - 2 * (q(2)*q(2) + q(3)*q(3));

    eul(3) = atan2(sinr_cosp, cosr_cosp);
    sinp = 2 * (q(1)*q(3) - q(4)*q(2));

    if (abs(sinp) >= 1)
        if sinp > 0
            eul(2) = pi/2;
        else
            eul(2) = -pi/2;
        end
    else
        eul(2) = asin(sinp);
    end

    siny_cosp = 2 * (q(1)*q(4) + q(2)*q(3));
    cosy_cosp = 1 - 2 * (q(3)*q(3) + q(4)*q(4));
    eul(1) = atan2(siny_cosp, cosy_cosp);
end
