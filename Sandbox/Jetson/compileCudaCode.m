deviceAddress = '192.168.55.1'; %SSH IP address0
userName = 'jetson';
password = 'jetson';
hwobj = jetson(deviceAddress,userName,password);

%camlist = getCameraList(hwobj);
%camName = table2array(camlist(1,"Camera Name"));
%camResolution = [1280 720];

envCfg = coder.gpuEnvConfig('jetson');
envCfg.BasicCodegen = 1;
envCfg.Quiet = 1;
envCfg.HardwareObject = hwobj;
coder.checkGpuInstall(envCfg);

cfg = coder.gpuConfig('exe');
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.Hardware.BuildDir = '~/build_matlab';
cfg.GenerateExampleMain = 'GenerateCodeAndCompile';
hwobj.setDisplayEnvironment('0.0');

%inputArgs = {coder.Constant(camName),coder.Constant(camResolution)};
codegen('-config', cfg, '-args', {}, 'nanoImgAndChecker', '-report');

pid = runApplication(hwobj,'nanoImgAndChecker');