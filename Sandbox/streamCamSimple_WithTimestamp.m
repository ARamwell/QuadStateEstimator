%Script to stream images from camera web server and save them to some
%specified folder

%% Initialise

n = 0;
nFrames = 100; 
url = 'http://192.168.175.121/capture';
saveDir = uigetdir();

%% Stream

disp('Recording in...');
disp('3');
pause(1);
disp('2');
pause(1);
disp('1');
pause(1)
disp('0');

startTime = datetime('now','Format','d-MMM-y HH:mm:ss:SSS');
while (n<nFrames)
    img = webread(url);

    timestamp_pc = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
%    imgHist(:,:,:,n) = img
%    imgHist_timestamps(n) = t;
%    imshow(img)

    fileName = ['esp32_' char(timestamp_pc) '.jpg'];
    %fileName = sprintf("%03d",n)+".jpg";
    imwrite(img, fullfile(saveDir,fileName), 'jpg','Comment', char(timestamp_pc));
    
    n=n+1;
end

%% Save

endTime = datetime('now','Format','d-MMM-y HH:mm:ss:SSS');
elapsedTime = duration((endTime - startTime), 'Format', 'mm:ss.SSS');
fps = nFrames/seconds(elapsedTime);
disp(strcat('Video recorded. (', string(nFrames), ') frames at (', string(fps), ') fps.'));
