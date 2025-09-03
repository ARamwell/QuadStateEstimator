function [img, timestamp_pc] = esp32_captureOnly(url, showImg)
%ESP32_CAPTUREANDUPDATEARRAY Summary of this function goes here
%   Detailed explanation goes here
    img = webread(url);
    
    timestamp_pc = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    
    %fileName = ['esp32_' char(timestamp_pc) '.jpg'];
    %imwrite(img, fullfile(saveDir,fileName), 'jpg','Comment', char(timestamp_pc));
    if showImg ==1
        imshow(img);
    end
end

