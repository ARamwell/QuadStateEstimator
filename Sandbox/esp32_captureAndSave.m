function esp32_captureAndSave(url,saveDir)
%ESP32_CAPTUREANDSAVE Summary of this function goes here
%   Detailed explanation goes here
    img = webread(url);
    
    timestamp_pc = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    
    fileName = ['esp32_' char(timestamp_pc) '.jpg'];
    imwrite(img, fullfile(saveDir,fileName), 'jpg','Comment', char(timestamp_pc));

end

