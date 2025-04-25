clear all


%fps = 2;
nFrames = 40;
n = 0;


%% STREAMING
%function img = stream()
%cam = ipcam('http://192.168.159.121:81/stream');
%cam = ipcam('http://192.168.175.121:81/stream'); %alyssawifi

%preview(cam)

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
    timestamp_pc = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss:SSS'); %
    %[img, timestamp_cam] = snapshot(cam); %capture image from stream
    %img= webread('http://192.168.175.121/capture'); %capture image from URL (no timestamp)
    imshow(img)
    %image(img);
    %step(hVideoIn,ss)
    n = n+1;
end

elapsedTime = toc;
fps = nFrames/elapsedTime;

%% FUNCTIONS FOR CAPTURE AND TIMESYNC
function [img, camTime, pcTime] = getESP32ImageWithTimestamp(url)
    % Example URL: 'http://192.168.1.123/capture'
    
    conn = java.net.URL(url).openConnection();
    conn.setRequestMethod('GET');
    conn.connect();

    % Read ESP32 timestamp
    headerStr = char(conn.getHeaderField('X-Camera-Timestamp'));
    camTime = datetime(headerStr, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss', 'TimeZone', 'UTC');

    % Read image stream
    inputStream = conn.getInputStream();
    imgData = readstream(inputStream);
    img = imdecode(uint8(imgData), 'jpg');

    % PC-side timestamp
    pcTime = datetime('now', 'TimeZone', 'UTC');
end

function data = readstream(is)
    import java.io.*;
    baos = ByteArrayOutputStream();
    buffer = zeros(1, 1024, 'int8');
    while true
        len = is.read(buffer, 0, 1024);
        if len == -1, break; end
        baos.write(buffer, 0, len);
    end
    data = typecast(baos.toByteArray(), 'uint8');
end




