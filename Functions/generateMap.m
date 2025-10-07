parentFile = pwd;
%%
%useful measurements from mocap, elsewhere
checkerCentre_pos_mocap = [149.49 2 106.09]/1000; %mm->m, from mocap
pos_fakeDrone2pixhawk_sw = [70.89 -50.39 -9.1]/1000; %mm->m, from Solidworks
R_fakeDrone2pixhawk_sw = eye(3);

R_fakeDroneEst2fakeDroneTrue = eul2rotm(deg2rad([8   1.5   0.5]), 'XYZ')';%quat2rotm([0.9979   -0.0607   -0.0236   -0.0030]); %q_av_est2gt, from estMisalignment
R_fakeDroneTrue2fakeDroneEst = R_fakeDroneEst2fakeDroneTrue';
R_fakeDrone2pixhawk = R_fakeDrone2pixhawk_sw * R_fakeDroneTrue2fakeDroneEst;
pos_fakeDrone2pixhawk = pos_fakeDrone2pixhawk_sw;


pos_esp2pixhawk = [-2.25 0 48.63]/1000; %mm->m, from Solidworks
% R_align = eul2rotm(deg2rad([8   1.5   0.5]), 'XYZ')'; %all static

%CURRENTLY ASSUMING DRONE CENTRE IS AT PIXHAWK CENTRE
%%
%useful transforms
T_nwu2ned = [1  0  0 0;
             0 -1  0 0;
             0  0 -1 0;
             0  0  0 1];
% 
% T_sim2world= [1 0 0 0; 
%               0 -1 0 0; 
%               0 0 1 0;
%               0 0 0 1]; %NWU
T_sim2world= [1 0 0 0; 
              0 -1 0 0; 
              0 0 -1 0;
              0 0 0 1]; %sim is neu?

T_world2sim = worldObject.invHomog(T_sim2world);

T_simcam2gencam = ([0 -1 0 0;
                    0 0 -1 0 ;
                    1 0  0 0 ;
                    0 0  0 1]);

T_simquad2genquad = ([1  0   0  0;
                      0  -1  0  0;
                      0  0  -1  0; 
                      0  0  0   1]);


T_imu2genquad = ([1 0 0 0;
                  0 1 0 0;
                  0 0 1 0; ...
                  0 0 0 1]); %ned


T_gencam2genquad = ([ [0 -1 0; 1 0 0; 0  0 1] pos_esp2pixhawk']);
T_gencam2genquad = [T_gencam2genquad; 0 0 0 1];
%rt_gencam2genquad = ([ [0 1 0; -1 0 0; 0  0 1] pos_esp2pixhawk']);


T_mocap2nwu = [0 0 -1 0;
              -1 0  0 0;
               0 1  0 0;
               0 0  0 1];
T_mocap2ned =   [0 0 -1 0;
                 1 0  0 0;
                 0 -1 0 0;
                 0  0 0 1];
T_mocap2world = T_mocap2ned;
 % rt_mocap2world = [0 1 -0 0;
 %                   -1 0 0 0;
 %                   0  0 1 0];

T_markers2genquad = [eye(3) pos_fakeDrone2pixhawk'];
T_markers2genquad = [T_markers2genquad; 0 0 0 1];


transformStruct = struct('T_sim2world', T_sim2world, 'T_simcam2gencam', T_simcam2gencam, 'T_simquad2genquad', T_simquad2genquad, 'T_imu2genquad', T_imu2genquad, 'T_gencam2genquad', T_gencam2genquad, 'T_mocap2world', T_mocap2world, 'T_markers2genquad', T_markers2genquad);
%transformStruct = struct('rt_sim2world', rt_sim2world, 'rotm_simcam2gencam', rotm_simcam2gencam, 'rotm_simquad2genquad', rotm_simquad2genquad, 'rt_imu2genquad', rt_imu2genquad, 'rt_gencam2genquad', rt_gencam2genquad, 'rt_mocap2world', rt_mocap2world, 'rt_mcquad2genquad', rt_mcquad2genquad);
%% 
% 

checkerSize = [5,8];
checkerSquareSize = 31; %mm
%checkerPos = [(((checkerSize(1)/2)-1)*checkerSquareSize) (((checkerSize(2)/2)-1)*checkerSquareSize) 1]; %mm
checkerPos = (T_mocap2world(1:3,1:3)*checkerCentre_pos_mocap')'; 
checkerOrient = [0 0 90];%rad2deg(rotm2eul([0 -1 0; -1 0 0; 0 0 1], 'XYZ'));% [-180 0 90]; %checker body in RW, where checker is ESD (like camera)

checkerObj = checker(checkerSize, checkerSquareSize, checkerPos, checkerOrient, "checkerboard", parentFile, T_sim2world);

%create list of all checkerboards
checkerList = [checkerObj];
%%
%April tag
aprilSize = 0.15; %m
%aprilObj1 = april(aprilSize, 1, [1,1,0.001], [-180 0 90], "april1", parentFile, T_sim2world); 
%aprilObj2 = april(aprilSize, 2, [1,2,0.001], [-180 0 90], "april2", parentFile, T_sim2world); 
%aprilObj3 = april(aprilSize, 3, [2,2,0.001], [-180 0 90], "april3", parentFile, T_sim2world); 

%aprilList = [aprilObj1, aprilObj2, aprilObj3];
aprilList = [];
%%
%room config
room_height = 2;
room_width = 5;%y dimension
room_length = 8;%x dimension

%walls
leftWallPos_nwu = [0, 1/2*room_width, 1/2*room_height];
leftWallOrient_nwu = [90,0,0];
leftWallT_nwu = [[eul2rotm(deg2rad(leftWallOrient_nwu), 'XYZ'), leftWallPos_nwu']; [0 0 0 1]];
leftWallT_ned= T_nwu2ned * leftWallT_nwu;
leftWallOrient_ned =  [90 0 0];%rad2deg(tform2eul(leftWallT_ned, 'XYZ'));
leftWallPos_ned = [0, -1/2*room_width, -1/2*room_height];
leftWallObj = wall(leftWallPos_ned, leftWallOrient_ned, [room_length, room_height, 0], "leftWall", ([255,240,219]/255), T_sim2world);

farWallPos_nwu =[1/2*room_length, 0, 1/2*room_height];
farWallOrient_nwu = [90,0,90];
farWallT_nwu = [[eul2rotm(deg2rad(farWallOrient_nwu), 'XYZ'), farWallPos_nwu']; [0 0 0 1]];
farWallT_ned= T_nwu2ned * farWallT_nwu;
farWallOrient_ned =  rad2deg(tform2eul(farWallT_ned, 'XYZ'));
farWallPos_ned = tform2trvec(farWallT_ned);
farWallObj =  wall(farWallPos_ned, farWallOrient_ned, [room_width, room_height, 0], "farWall", ([255,240,219]/255), T_sim2world);

%create list of all walls
wallList = [leftWallObj, farWallObj];

%floor
green = ([34,139,34]/255);
grey = [187 189 124]/255;
floorObj = stage([0,0,0], [0,0,0], [room_length, room_width, 0], "floor", grey, T_sim2world);

%create list of all floors
floorList = [floorObj];
%%
%create struct of all objects
worldObjectStruct= struct('transforms', transformStruct, 'walls', wallList, 'checkers', checkerList, 'floors', floorList, 'aprils', aprilList);

%save to .mat filefullfile('.', '/QuadSimEnv/Results/Traj-0005/fps_20');
save(fullfile('.', '/Resources/map'), "worldObjectStruct");
%save("C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Pnp_solver\GitClone\Resources\map", "worldObjectStruct");