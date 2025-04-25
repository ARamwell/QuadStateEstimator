function imgArray = dataCaptureWorker_cam(dq1)
% Accepts data queue

    %send(dq, 'Cam Worker started');

    url = 'http://192.168.175.121/capture';
    saveDir = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Sandbox\CamImgs\simpleStream';
    imgArray = zeros(240,320,3, 1);

    disp('Camera capture ready');

    % Wait for 'start' trigger
    % while true
    %     if dq.QueueLength > 0
    %         msg = poll(dq, 1);
    %         if strcmp(msg, 'start')
    %             break;
    %         end
    %     end
    % end
    
    %send(dq, 'Cam Worker capture START triggered');
   

    % % Loop until 'stop' trigger received
    % while true
    %     % Capture data here
    %     esp32_captureAndSave(url, saveDir);
    % 
    %     %pause(0.3); %placeholder
    % 
    %     if dq.QueueLength > 0
    %         msg = poll(dq);
    %         if strcmp(msg, 'stop')
    %             break;
    %         end
    %     end
    % end
    for n=1:2
        %esp32_captureAndSave(url, saveDir);
        img = webread(url);
        imgArray(:,:,:,n) = img;
        imshow(img);
        pause(0.3);
    end

    %send(dq, 'Cam Worker capture STOP triggered');
end
