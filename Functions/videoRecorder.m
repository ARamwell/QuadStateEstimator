
%function img = stream()
cam = ipcam('http://192.168.159.121:81/stream');

%preview(cam)


%imshow(img)
saveDir = uigetdir();
%fps = 2;
nFrames = 40;
%recordVideo('http://192.168.225.121:81/stream', 1, 1, folder)
%recordVideo(cam, 1, 1, folder)


n = 0;
pause(3)
disp('Recording in...');
disp('3');
pause(1);
disp('2');
pause(1);
disp('1');
pause(1)
disp('0');
tic
while (n<nFrames) 
%    ss=imread(url);
%    imshow(ss);
    
    fileName = sprintf("%03d",n)+".jpg";
    [img, timestamp] = snapshot(cam); %capture image
    imshow(img)
    pause(0.3)
    %image(img);
    imwrite(img, fullfile(saveDir,fileName), 'jpg','Comment', string(timestamp));
    %step(hVideoIn,ss)
    n = n+1;
end

elapsedTime = toc;
fps = nFrames/elapsedTime;
disp(strcat('Video recorded. (', string(nFrames), ') frames at (', string(fps), ') fps.'));

clear all


