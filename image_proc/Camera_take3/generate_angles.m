function angles = generate_angles(N, sigma)

    pd = makedist('Normal','mu',0,'sigma',sigma);

    ang = zeros(1,N);
    for i=1:N
        ang(i) = pd.random;
    end
    
    figure;
    histfit(ang, N,'Normal');
    
    ang_rad = ang*pi/180;
     
    angles(1      :N,:)  = [ang_rad;zeros(1,N);zeros(1,N)]';
    angles((N+1)  :2*N,:) = [zeros(1,N);ang_rad;zeros(1,N)]';
    angles((2*N+1) :3*N,:) = [zeros(1,N);zeros(1,N);ang_rad]';
        
end