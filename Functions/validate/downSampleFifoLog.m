
%% %Select a .mat file
[filename, pathname] = uigetfile('*.mat', 'Select a MAT file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

%% Import data
imuData = load(filepath);

if isstruct(imuData)
    fields = fieldnames(imuData);
    imuData = getfield(imuData, fields{1});
end

if (size(imuData, 2) > 6)
    imuData = imuData';
end

%% Get downsampler settings
%rawHz = 8.3705e+04; %from parsed accel fifo; 
rawHz = 6e+03; %from parsed gyro FIFo
newHz = 250;

numReadings = size(imuData, 1);
decimation = floor(rawHz/newHz);
accumData = zeros(1,size(imuData, 2));
lowRateData = createArray(1, size(imuData, 2));
s=1;

for i=1:numReadings
    newData = imuData(i,:);
    
    if s<decimation
        s = s+1;
        accumData = accumData + newData;
    else
        lowRateData(end+1,:) = accumData/s;
        accumData = newData;
        s=1;
    end
end


    

