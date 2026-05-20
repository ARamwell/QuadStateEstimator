function nanoCamCalib()
%NANOCAMCALIB Capture some images and run camera calibration
%   Detailed explanation goes here

boardSize = [5, 8];
%camRes = [1280 720];
camRes = [640 360];
squareSizeInMM = 31;
numImgs = 20;
%imgs = createArray(camRes(1), camRes(2), 3, numImgs);
numPts = (boardSize(1)-1)*(boardSize(2)-1);
imagePoints_comp = createArray(2, numPts, numImgs);
%worldPoints_comp = createArray((boardSize(1)*(boardSize(2))), 3, numImgs);
counter = 0;

hwobj = jetson;
camName = 'vi-output, imx219 6-0010';
camObj = camera(hwobj,camName,camRes);
dispObj = imageDisplay(hwobj); %for troubleshooting

for i = 1:numImgs
    %capture image
    img_i = rot90(snapshot(camObj), 2);
    
    %display
    image(dispObj, img_i);

    %detect checkerboard
    [imagePoints, boardSize_det] = detectCheckerboardPoints(img_i);

    %add to arrays
    if (boardSize_det(1)*boardSize_det(2)) == (boardSize(1)*boardSize(2))
        counter = counter + 1;
        imagePoints_comp(:,:,counter) = imagePoints';
        
    end
        
end

worldPoints = patternWorldPoints("checkerboard",boardSize,squareSizeInMM);

imageSize = [size(img_i, 1),size(img_i, 2)];
%params = estimateCameraParameters(imagePoints,worldPoints, ...
%                              'ImageSize',imageSize);

%fileID = fopen("/home/ros2_ws/outputs/cameraCalibData.txt", 'w');
fileID = fopen("cameraCalibData.txt", 'w');
%fileID = fopen('C:\Users\Alyssa\Downloads\test.txt', 'w');

%fprintf(fileID, '%5d %5d %5d \n', params.K');
%print worldPoints
for j =1:2
    for n=1:size(worldPoints, 1)
        fprintf(fileID, '%f,', worldPoints(n, j));
    end
    fprintf(fileID, '\n');
end

%PRINT IMAGE SIZE
fprintf(fileID, '%f,', imageSize(1));
fprintf(fileID, '%f', imageSize(2));
fprintf(fileID, '\n');

%print image points
for i=1:numImgs 
    for j = 1:size(imagePoints_comp, 1) %rows
        for n=1:size(imagePoints_comp, 2) %columns
            fprintf(fileID, '%f,', imagePoints_comp(j, n, i));
        end
        fprintf(fileID, '\n');
    end
end
fclose(fileID);

end
