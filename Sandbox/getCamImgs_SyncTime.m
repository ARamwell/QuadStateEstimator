import matlab.net.*
import matlab.net.http.*


%fps = 2;
nFrames = 40;
n = 0;
url = 'http://192.168.175.121/capture';



disp('Recording in...');
disp('3');
pause(1);
disp('2');
pause(1);
disp('1');
pause(1)
disp('0');

while (n<nFrames) 
%    ss=imread(url);
%    imshow(ss);
    
    img = webread(url);
    imshow(img);
   
    % fileName = sprintf("%03d",n)+".jpg";
    timestamp_pc_1 = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss:SSS'); %
    % [img, timestamp_cam, timestamp_pc] = getESP32ImageWithTimestamp(url);
   
    % Send HTTP GET request
    request = RequestMessage;
    response = request.send(url);
    timestamp_pc_2 = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss:SSS'); %
    
    % Get the JPEG image data
    %img = imread(uint8(response.Body.Data), 'jpg');
    img = uint8(response.Body.Data);
    
    
    % Show the image
    imshow(img);

    % Get the timestamp header
    timestamp = response.getFields('X-Camera-Timestamp');
    disp("ESP32 timestamp: " + string(timestamp.Value));
    disp("PC timestamp 1: " + string(timestamp_pc_1));
    disp("PC timestamp 2: " + string(timestamp_pc_2));
    %timeDiff = str2double(timestamp_pc_1 - timestamp_pc_2)/2 - str2double(timestamp.Value);
    %disp(timeDiff);

    % Process and display usefully
    RTT = duration((timestamp_pc_2 - timestamp_pc_1), 'Format', 'mm:ss.SSS');
    delay = duration((timestamp_pc_1 - timestamp.Value), 'Format', 'mm:ss.SSS');
    disp("Round trip time: " + string(RTT));
    disp("Delay between request and camera timestamp: " + string(delay));
    disp(".");

        
    n = n+1;
end

%fps = nFrames/elapsedTime;

% %% FUNCTIONS FOR CAPTURE AND TIMESYNC
% function [img, camTime, pcTime] = getESP32ImageWithTimestamp(url)
%     % Example URL: 'http://192.168.1.123/capture'
% 
%     conn = java.net.URL(url).openConnection();
%     conn.setRequestMethod('GET');
%     conn.connect();
% 
%     % Read ESP32 timestamp
%     headerStr = char(conn.getHeaderField('X-Camera-Timestamp'));
%     camTime = datetime(headerStr, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss', 'TimeZone', 'UTC');
% 
%     % Read image stream
%     inputStream = conn.getInputStream();
%     imgData = readstream(inputStream);
%     img = imdecode(uint8(imgData), 'jpg');
% 
%     % PC-side timestamp
%     pcTime = datetime('now', 'TimeZone', 'UTC');
% end
% 
% function data = readstream(is)
%     import java.io.*;
%     baos = ByteArrayOutputStream();
%     buffer = zeros(1, 1024, 'int8');
%     while true
%         len = is.read(buffer, 0, 1024);
%         if len == -1, break; end
%         baos.write(buffer, 0, len);
%     end
%     data = typecast(baos.toByteArray(), 'uint8');
% end
% 
% 
% 
