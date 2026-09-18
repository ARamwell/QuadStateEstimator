function [trajOut, trajNames, simset] = simGetWP(simset, trajIn)
%SIMGETTRAJ Summary of this function goes here

if simset.mocapTraj == false %then manual input or some standard trajectories
    %then trajIn will specify what we want

    if ~isstring(trajIn)
        %then it is an array

    else %specify an existing trajectory
   
        if strcmp(trajIn, "simpleInf")
            %Infinity trajectory
            trajNames = "simpleInf";
            in_times = [0.01, 0.3, 3, 5, 7, 9];
            %wp_pos = [t_checker; t_checker; 0 0 -1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
            %wp_pos = [0 0 0; 0 0 0; t_checker(1:2) -1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
            in_pos = [0 0 0; 0 0 0; 0 0.2 -1.1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
            in_eul = [-0 0 0; 0 0 0; 0 0 0; 20 -20 0; -20 0 10; 0 0 0]';
            cut = false;
        
        elseif strcmp(trajIn, "elev8")
            %elevated figure eight
            trajNames = "elev8";
            in_times = [0.01, 0.3, ...
                3, 5, 7, 9, 11]';
            in_pos = [0 0 -1.1; 0 0 -1.1; 0.3 -0.5 -1.1; 0 -0 -1.4; 0.7 0.7 -1.6; -0.5 0.7 -1.4;  0 0 -1.1]';
            in_eul = [0 0 0; 0 0 0;0 -21 -25;  -10 0 -30; 20 -20 0; 15 -5 50;  0 0 0]';
            cut = false;
 

        elseif strcmp(trajIn, "static")
            %elevated figure eight
            trajNames = "static";
            in_times = [0.01, 0.3, 30];
            in_pos = [0 0 -1; 0 0 -1; 0 0 -1]';
            in_eul = [0 0 0; 0 0 0;0 0 0]';
            cut = false;

        elseif strcmp(trajIn, "staticExtreme")
            %elevated figure eight
            trajNames = "staticExtreme";
            %in_times = [0.01, 0.3, 30];
            %in_pos = [1 -1 -1.5; 1 -1 -1.5; 1 -1 -1.5]';
            %in_eul = [-45 -15 0; -45 -15 0;-45 -15 0]';
            in_times = [0.01, 0.3, 5];
            pos = [1.74 -0.4315 -1.4419];
            in_pos = [pos; pos; pos]';
            eul =rad2deg(quat2eul([0.8519, -0.1708, -0.4477, 0.2125], 'XYZ')); 
            in_eul = [eul; eul; eul]';
            cut = false;
        
        elseif contains(trajIn, "spiral")
            %Archimedes spiral trajectory with camera pointing at target [0 0 0]
            %Parameters can be passed as: "spiral_rpm_H_dr_dz_sh_sr_nw" where:
            %  rpm: rotations per minute (default 1)
            %  H: number of loops/rotations (default 3)
            %  dr: horizontal distance increase per loop in meters (default 0.5)
            %  dz: vertical distance increase per loop in meters (default 0.3)
            %  sh: starting height above target in meters (default 0.5)
            %  sr: starting radius in meters (default 0.1)
            %  nw: number of waypoints (default 30)
            %Example: "spiral_1_3_0.5_0.3_0.5_0.1_30"
            
            trajNames = "spiral";
            
            % Parse parameters from trajIn string
            % parts = strsplit(trajIn, '_');
            % if length(parts) >= 2
            %     rpm = str2double(parts{2});
            %     if isnan(rpm), rpm = []; end
            % else
            %     rpm = [];
            % end
            % if length(parts) >= 3
            %     numLoops = str2double(parts{3});
            %     if isnan(numLoops), numLoops = []; end
            % else
            %     numLoops = [];
            % end
            % if length(parts) >= 4
            %     dr_per_loop = str2double(parts{4});
            %     if isnan(dr_per_loop), dr_per_loop = []; end
            % else
            %     dr_per_loop = [];
            % end
            % if length(parts) >= 5
            %     dz_per_loop = str2double(parts{5});
            %     if isnan(dz_per_loop), dz_per_loop = []; end
            % else
            %     dz_per_loop = [];
            % end
            % if length(parts) >= 6
            %     startHeight = str2double(parts{6});
            %     if isnan(startHeight), startHeight = []; end
            % else
            %     startHeight = [];
            % end
            % if length(parts) >= 7
            %     startRadius = str2double(parts{7});
            %     if isnan(startRadius), startRadius = []; end
            % else
            %     startRadius = [];
            % end
            % if length(parts) >= 8
            %     numWaypoints = str2double(parts{8});
            %     if isnan(numWaypoints), numWaypoints = []; end
            % else
            %     numWaypoints = [];
            % end
            
            % Generate spiral trajectory using helper function
            % Parameters: rpm, numLoops, dr_per_loop, dz_per_loop, startHeight, startRadius, numWaypoints, numRollRotations
            % numRollRotations: number of full 360° roll rotations across the trajectory (default 0)
            [in_times, in_pos, in_eul] = generateSpiralTrajectory(...
                5, 3, 0.5, 0.2, ...
                0.8, 0.01, 200, 2);  % Set numRollRotations to 0 for no roll, or e.g., 2 for 2 full rotations
            cut = true;

        elseif contains(trajIn, "SettleAndWiggle", 'IgnoreCase', true)
            % ~60 s: move (0-20 s), hold position + wiggle attitude (20-60 s)
            trajNames = "SettleAndWiggle";
            in_pos =   [0 0 -1.8;
                        0 0 -1.8;
                        1, 1, -1.8; %top right
                        -1, 1, -2; %bottom right
                        1, -1, -2;%top left
                        0 0 -1.8;  %centre
                        0 0 -1.8; %rotating
                        0 0 -1.8; %rotating
                        0 0 -1.8]'; %rotating
            in_eul =   [0 0 0;
                        0 0 0;
                        30 -15 0;
                        25 15 0;
                        -20 -10, 0;
                        0 0 0;
                        -130, 95, -15;
                       45 -20 150
                       150 -120 0]';
            in_times = [0.01, 5, 10, 15, 20, 30, 40, 50, 60];
             % [in_times, in_pos, in_eul] = generateSettleAndWiggleTrajectory();
            % simset.trajInterp = "linear";
            % cut = false;

        elseif contains(trajIn, "ExciteAndWiggle", 'IgnoreCase', true)
            
            trajNames = "ExciteAndWiggle";
            
            target = [0; 0; 0];
            
            in_times = [ ...
                0.01, 4, ...
                7, 10, 13, 16, 19, 22, 25, 28, ...
                31, 35];
            
            in_pos = [ ...
                 0.00,  0.00, -1.80;  % settle
                 0.00,  0.00, -1.80;  % settle
            
                 0.65,  0.00, -1.55;  % +x, shallow
                 0.25,  0.65, -2.05;  % +x/+y, deep
                -0.65,  0.20, -1.65;  % -x
                -0.25, -0.65, -2.15;  % -x/-y, deep
                 0.75, -0.25, -1.70;  % +x/-y
                 0.00,  0.75, -1.50;  % +y, shallow
                -0.75,  0.00, -2.00;  % -x, deep
                 0.00, -0.75, -1.60;  % -y, shallow
            
                 0.45,  0.45, -2.20;  % final diagonal depth change
                 0.00,  0.00, -1.80]'; % return before your static rotation phase
            
            roll_about_view = [0, 0, 20, -25, 35, -35, 45, -45, 25, -20, 40, 0];
            
            in_eul = zeros(3, numel(in_times));
            for k = 1:numel(in_times)
                in_eul(:, k) = eulerLookAtTarget(in_pos(:, k), target, roll_about_view(k));
            end

            statPhase_pos =  [0.00,  0.00, -1.80; 
                             0.00,  0.00, -1.80;  
                             0.00,  0.00, -1.80]';
            statPhase_eul =  [-130, 95, -15;
                                45 -20 150;
                                150 -120 0]';
            statPhase_t = [15, 30, 45];

            in_pos = [in_pos statPhase_pos];
            in_eul = [in_eul, statPhase_eul];
            in_times=[in_times, (in_times(end)+statPhase_t)];

        end

        simset.duration =in_times(end);
        % [wp_pos, ~, ~, ~, ~, ~, ~, wp_times] = minsnappolytraj(in_pos, in_times, (simset.duration*simset.simHz));
        % wp_eul = minsnappolytraj(in_eul, in_times, (simset.duration*simset.simHz));
        % 
        % if cut
        %     cutDur = 2; %ut off last 2 seconds
        %     numWp = cutDur*simset.simHz;
        %     wp_pos = wp_pos(:,1:end-numWp);
        %     wp_eul = wp_eul(:,1:end-numWp);
        %     wp_times = wp_times(:,1:end-numWp);
        %     simset.duration = simset.duration-cutDur;
        % end

    end

    
    trajOut = [in_times; in_pos; in_eul]; 
    

else

    R_align = eye(3);
    T_mc2rw = simset.map.worldObjectStruct.transforms.T_mocap2world;
    T_mcq2rq = simset.map.worldObjectStruct.transforms.T_markers2genquad;
    T_mcq2rq(1:3, 1:3) = T_mcq2rq(1:3, 1:3)*R_align;  
    T_uq2rq = simset.map.worldObjectStruct.transforms.T_simquad2genquad;

    %if isstring(trajIn) %then it is a folder structure

        sourceFolder = trajIn;

        if simset.multisim == true
            [listOfFolderNames] = selector_multiFolder();
        %     % Find all subfolders
        %     allSubFolders = genpath(sourceFolder);
        %     % Parse into a cell array.
        %     remain = allSubFolders;
        %     listOfFolderNames = {};
        %     while true
	    %         [singleSubFolder, remain] = strtok(remain, ';');
	    %         if isempty(singleSubFolder)
		%             break;
        %         end
        %         if ~contains(singleSubFolder, 'arb') && ~contains(singleSubFolder, 'calib') && contains(singleSubFolder, 'tatic') && ~contains(singleSubFolder, 'sim')
	    %             listOfFolderNames = [listOfFolderNames singleSubFolder];
        %         end
        %     end
        %     listOfFolderNames = listOfFolderNames(2:end);
             numberOfSubFolders = length(listOfFolderNames);
        else
            listOfFolderNames = sourceFolder;
            numberOfSubFolders = 1;
        end

        for k=1:numberOfSubFolders
            if numberOfSubFolders ==1 
                currentFolder = string(listOfFolderNames);
            else
                currentFolder = string(listOfFolderNames(k));
            end
        
            %get name
            parts = strsplit(currentFolder, '\');
            trajName = string((parts{end}));

            %extract recorded trajectory
            mocapLogFolder = (fullfile(currentFolder, 'mocapLog_sync.mat'));
            [mocap.quadRt, mocap.quadState, mocap.quadTime, mocap.elapsedTime] = importMocapLog(mocapLogFolder, T_mc2rw, T_mcq2rq);
            
            
            %extract time limits
            startTime = datetime(mocap.quadTime(1), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
            endTime = datetime(mocap.quadTime(end), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
            simset.duration = milliseconds(endTime - startTime)/1000;
            
            %R_corr = [-1 0 0; 0 -1 0; 0 0 -1];
        
            %build time series trajectory for sim
            mc_times = milliseconds(mocap.quadTime(:)-startTime)/1000;
            mc_pos = mocap.quadState(1:3, :);
            mc_eul = ((quat2eul((mocap.quadState(4:7, :)'), 'XYZ')))';   
            skip =50; %make the array smaller and smoother by skipping a few values
            wp_times = mc_times(1:skip:end,:); % [mc_times(1:5,:); mc_times(6:skip:end,:)];
            wp_pos =  mc_pos(:, 1:skip:end); %[mc_pos(:,1:5), mc_pos(:, 6:skip:end)];
            wp_eul =  mc_eul(:, 1:skip:end);%[mc_eul(:,1:5), mc_eul(:, 6:skip:end)];  
            wp_eul = rad2deg(unwrap(wp_eul, [], 2));
            %wp_eul = smoothdata(wp_eul, 2,"rlowess", 5);
            %wp_pos = smoothdata(wp_pos, 2,"rlowess", 5);

            %offset z-position to keep target in sight
            wp_pos = wp_pos + [0 0 -0.4]';
                
            %simset.duration =wp_times(end);
            % [wp_pos, ~, ~, ~, ~, ~, ~, wp_times] = minsnappolytraj(wp_pos, wp_times, ceil(simset.duration*simset.simHz));
            % wp_eul = minsnappolytraj(wp_eul, wp_times, ceil(simset.duration*simset.simHz));

        
            %figure();
            %plot(wp_times, wp_eul);
        
            %convert to timeseries
            %traj_pos_ts = timeseries(wp_pos, wp_times);
            %traj_eul_ts = timeseries(wp_eul, wp_times);
            %add to array (padding if necessary)
            newTraj = [wp_times'; wp_pos; wp_eul]; 
            if k>1
                lengthOld = size(trajOut, 2);
                lengthNew = size(newTraj, 2);
                if lengthOld > lengthNew
                    newTraj = padarray(newTraj, [0, max(0, lengthOld - lengthNew)], NaN, 'post');
                elseif size(trajOut, 2) < size(newTraj, 2)
                    trajOut = padarray(trajOut, [0, max(0,lengthNew - lengthOld), 0], NaN, 'post');
                end
            end
            trajOut(:, :, k) = newTraj; 
            trajNames(k) = trajName;
        end  
    end

%end


end

