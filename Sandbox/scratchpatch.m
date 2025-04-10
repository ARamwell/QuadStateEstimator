function frankData = frankensteinData();
    %Build frankenstein dataset
    
     
    filepath1 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_0951';
    filepath2 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_1014';
    filepath3 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_1029';
    
    
    data1 = readmatrix(filepath1); 
    data2 = readmatrix(filepath2); 
    data3 = readmatrix(filepath3); 
    
    frankData = [data1; data2; data3];
    
    writematrix(frankData, fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib/imuHist_imu2_static_raw_20Hz_20250317_0951-1014-1029.csv'));
end


function R_ = findImuMounting();

     
    filepath1 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\Px6C_imuHist_static_xdown_room2_raw_20250404_1102';
    filepath2 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\Px6C_imuHist_static_yup_room2_raw_20250404_1102';
    filepath3 = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\Px6C_imuHist_static_zdown_room2_raw_20250404_1102';
    
    data1_row = (readmatrix(filepath1)); 
    data2_row = (readmatrix(filepath2)); 
    data3_row = (readmatrix(filepath3));

    data1 = data1_row(:, 2:4)'; 
    data2 = data2_row(:, 2:4)'; 
    data3 = data3_row(:, 2:4)';

    data1 = imuCorrect(data1); 
    data2 = imuCorrect(data2); 
    data3 = imuCorrect(data3);

    g1 = [-9.7952; 0; 0];
    g2 = [0; 9.7952; 0];
    g3 = [0; 0; -9.7952];

    a1 = mean(data1,2);
    a2 = mean(data2, 2);
    a3 = mean(data3, 2);

    R = [g1 g2 g3] * inv([a1 a2 a3]);

    c1 = R(:,1);
    c2 = R(:,2);
    c3 = R(:,3);
    
    %c3 > c1 > c2
    c3_ = c3;
    c1_ = cross(c2, c3_);
    c2_ = cross(c3_, c1_);
    c1_ = c1_/norm(c1_); %normalise
    c2_ = c2_/norm(c2_); %normalise
    c3_ = c3_/norm(c3_); %normalise
    R_ = [c1_ c2_ c3_];
    disp(R_);




end 