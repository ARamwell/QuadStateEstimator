deviceAddress = '192.168.55.1'; %SSH IP address
userName = 'jetson';
password = 'jetson';

% create connection
%hwobj = jetson(deviceAddress,userName,password);
%hwobj_ = ros2device(deviceAddress,userName,password);
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')

% configure coder
cfg = coder.config('exe');
cfg.Hardware = coder.hardware("Robot Operating System 2 (ROS 2)");
%cfg.Hardware.ROS2Workspace = '~/ros2_px4_ws';
cfg.Hardware.ROS2Workspace = '/home/jetson/ros2_ws';
%cfg.Hardware.ROS2Folder = '/opt/ros/humble' ;
cfg.Hardware.ROS2Folder = '/home/jetson/ros2_humble/install' ;
cfg.Hardware.BuildAction = 'Build and Load';
cfg.Hardware.DeployTo = 'Remote Device';
cfg.GenCodeOnly = false;
%cfg.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM 64-bit (LP64)';
cfg.HardwareImplementation.ProdLongLongMode = true;
cfg.Hardware.RemoteDeviceAddress = deviceAddress;
cfg.Hardware.RemoteDeviceUsername = userName;
cfg.Hardware.RemoteDevicePassword = password;
%hwobj.setDisplayEnvironment('0.0');

%generate code
codegen('-config', cfg, '-args', {}, 'nanoImgCapture', '-report');
%codegen('-config', cfg, '-args', {}, 'ros2Listener_uxrce', '-report');
%codegen('-config', cfg, '-args', {}, 'helloWorld', '-report');

%run code on the board
%out = runApplication(hwobj,'ros2Listener_uxrce');