
%
%parpool(2);

% Create data queue for start and stop triggers
dq1 = parallel.pool.PollableDataQueue;
dq2 = parallel.pool.PollableDataQueue;

%Create workers
p1 = parfeval(@dataCaptureWorker_cam, 1, dq1);
%p2 = parfeval(@dataCapture_imu,  0, dq2);

pause(3); %Let workers start up

%Send start trigger
disp("Agents ready. Press key to start data capture.")
%pause;  %Wait for button press
pause(2)
disp("Triggering data capture to START now...")
send(dq1, 'start');
send(dq2, 'start');

%Wait a bit to "debounce"
pause(10);

%Send stop trigger
%pause; %Wait for button press
disp("Triggering data capture to STOP now...")
send(dq1, 'stop');
send(dq2, 'stop');


%% FUNCTIONS

%% Camera capture
function dataCapture_cam(dq)
% Accepts data queue

    url = 'http://192.168.175.121/capture';
    saveDir = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Sandbox\CamImgs\simpleStream';

    disp('Camera capture ready');

    % Wait for 'start' trigger
    while true
        msg = poll(dq, 1);
        if strcmp(msg, 'start')
            break;
        end
    end
    
    disp("Camera START triggered!")

    % Loop until 'stop' trigger received
    while true
        % Capture data here
        esp32_captureAndSave(url, saveDir);

        %pause(0.3); %placeholder

        if dq.QueueLength > 0
            msg = poll(dq);
            if strcmp(msg, 'stop')
                break;
            end
        end
    end

    disp("Camera STOP triggered!")

end

%% IMU capture
function dataCapture_imu(dq)
% Accepts data queue

    % Initialisations
        url = 'http://192.168.175.121/capture';
    saveDir = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Sandbox\CamImgs\simpleStream';

    disp('IMU capture ready');
    % 
    % % Wait for 'start' trigger
    % while true
    %     msg = poll(dq, Inf);
    %     if strcmp(msg, 'start')
    %         break;
    %     end
    % end
    
    disp("Camera START triggered!")
    % 
    % % Loop until 'stop' trigger received
    % while true
    %     % Capture data here
    %     %esp32_captureAndSave(url, saveDir);
    %     imwrite(img, fullfile(saveDir,fileName), 'jpg');
    %     pause(0.3); %placeholder
    % 
    %     if dq.QueueLength > 0
    %         msg = poll(dq);
    %         if strcmp(msg, 'stop')
    %             break;
    %         end
    %     end
    % end

    disp("IMU STOP triggered!")

end

function dispData(data)
    disp("Worker data: ", data);
end