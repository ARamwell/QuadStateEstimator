%function streamCamSimple_WithTimestamp(url, nFrames, saveFolder)

    %Script to stream images from camera web server and save them to some
    %specified folder
    
    %% Initialise
    
    
    %url = 'http://192.168.181.121/capture';
    url = 'http://192.168.0.103/capture';
    nFrames = 400;
    %saveDir = uigetdir();
    saveFolder = fullfile('.', '/Tests/RobMech/Dynamic/TestSeries_3/');
    
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
    
    startTime = datetime('now','Format','yyyyMMdd_HHmmss_SSS');
    imgArr = [];
    timestampArr = createArray(1, 0, 'datetime');
    timestampArr = datetime(timestampArr, 'Format', 'yyyyMMdd_HHmmss_SSS');
    
    n = 1;
    while (n<=nFrames)
        %esp32_captureAndSave(url, saveFolder, 0);
        [imgArr(:,:,:,n), timestampArr(1,n)]= esp32_captureOnly(url,0);
        
        n=n+1;   
    end
    endTime = datetime('now','Format','yyyyMMdd_HHmmss_SSS');
    
    %% Save
    for t=1:size(imgArr,4)
        img = imgArr(:,:,:,t);
        ts = timestampArr(1,t);
        fileName = ['esp32_' char(ts) '.jpg'];
        imwrite(uint8(img), fullfile(saveFolder,fileName), 'jpg','Comment', char(char(ts)));
        % if showImg ==1
        %     imshow(img);
        % end
    end
        
    
    %% Print interesting info
    elapsedTime = duration((endTime - startTime), 'Format', 'mm:ss.SSS');
    fps = nFrames/seconds(elapsedTime);
    disp(strcat('Video recorded. (', string(nFrames), ') frames at (', string(fps), ') fps.'));
