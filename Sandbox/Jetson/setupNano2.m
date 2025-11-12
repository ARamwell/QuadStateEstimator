deviceAddress = '192.168.55.1'; %SSH IP address
userName = 'jetson';
password = 'jetson';

% create connection
hwobj = jetson(deviceAddress,userName,password);

% configure coder
cfg = coder.config('exe');
cfg.TargetLang = 'C++';
%hwobj.setupCodegenContext; %don't actually need to run this if only one live connection (as in this example)
cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
cfg.Hardware = coder.hardware('NVIDIA Jetson');
nanoBuildDirectory = 'build_matlab';
cfg.Hardware.BuildDir = strcat('~/', nanoBuildDirectory); %set build directory. Will create a folder if it does not exist
%cfg.CustomSource  = fullfile('gsMain.cpp');

%generate code
%codegen('-config ',cfg,'helloWorld', ','','-report');
codegen('-config', cfg, '-args', {}, 'helloWorld', '-report');


%run code on the board
out = runApplication(hwobj,'helloWorld');