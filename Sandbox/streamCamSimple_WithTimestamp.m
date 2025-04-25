%Script to stream images from camera web server and save them to some
%specified folder

%% Initialise

n = 0;
nFrames = 100; 
url = 'http://192.168.175.121/capture';
saveDir = uigetdir();

%% Stream
% 
% disp('Recording in...');
% disp('3');
% pause(1);
% disp('2');
% pause(1);
% disp('1');
% pause(1)
% disp('0');

startTime = datetime('now','Format','d-MMM-y HH:mm:ss:SSS');
while (n<nFrames)
    esp32_captureAndSave(url, saveDir);
    n=n+1;
end

%% Save

endTime = datetime('now','Format','d-MMM-y HH:mm:ss:SSS');
elapsedTime = duration((endTime - startTime), 'Format', 'mm:ss.SSS');
fps = nFrames/seconds(elapsedTime);
disp(strcat('Video recorded. (', string(nFrames), ') frames at (', string(fps), ') fps.'));
