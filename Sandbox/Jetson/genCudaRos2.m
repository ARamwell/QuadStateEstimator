deviceAddress = '192.168.55.1'; %SSH IP address
userName = 'jetson';
password = 'jetson';

hwobj = jetson(deviceAddress,userName,password);

% Get camera details
camlist = getCameraList(hwobj);
camName = table2array(camlist(1,"Camera Name"));
camResolution = [1280 720];

% % Configure gpu coder and check it
% envCfg = coder.gpuEnvConfig('jetson');
% envCfg.BasicCodegen = 1;
% envCfg.BasicCodeexec = 1;
% envCfg.Quiet = 1;
% envCfg.HardwareObject = hwobj;
% results = coder.checkGpuInstall(envCfg)

 %% Configure hardware deployment - ros2 
cfg = coder.config('exe');

%cfg = coder.gpuConfig('exe');
cfg.Hardware = coder.hardware("Robot Operating System 2 (ROS 2)");
cfg.CppPreserveClasses = false;
cfg.Hardware.DeployTo = 'Remote Device';
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM 64-bit (LP64)';
cfg.HardwareImplementation.ProdLongLongMode = true;
%cfg.Hardware.CUDAStandard = '14';  % <—— this line!

cfg.Hardware.ROS2Workspace = '/home/jetson/ws_ros2_px4';
cfg.Hardware.ROS2Folder = '/home/jetson/ros2_humble/install' ;
% cfg.Hardware.BuildAction = 'Build and run';
% %cfg.Hardware.BuildAction = 'Build and Load';
% 
% cfg.GenCodeOnly = false;

cfg.Hardware.RemoteDeviceAddress = deviceAddress;
cfg.Hardware.RemoteDeviceUsername = userName;
cfg.Hardware.RemoteDevicePassword = password;

%More GPU config?
cfg.GpuConfig = coder.GpuCodeConfig;
cfg.GpuConfig.Enabled = true;

%% Non ROS2 deployment - any better?
% cfg = coder.gpuConfig('exe');
% cfg.Hardware = coder.hardware('NVIDIA Jetson');
% nanoBuildDirectory = 'build_matlab';
% cfg.Hardware.BuildDir = strcat('~/', nanoBuildDirectory); %set build directory. Will create a folder if it does not exist
% cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
hwobj.setDisplayEnvironment('0.0');

% Run!
%inputArgs = hwobj, camName, camResolution
inputArgs = {coder.Constant(camName),coder.Constant(camResolution)};
codegen('-config', cfg, '-args', {}, 'nanoImgCapture', '-report');

%pid = runApplication(hwobj,'nanoImgCapture');