function [trajOut, trajNames, simset] = simGetTraj(simset, trajIn)
%SIMGETTRAJ Summary of this function goes here

if simset.mocapTraj == false %then manual input or some standard trajectories
   
    %Infinity trajectory
    trajNames = 'simpleInf';
    in_times = [0.01, 0.3, 3, 5, 7, 9];
    %wp_pos = [t_checker; t_checker; 0 0 -1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
    %wp_pos = [0 0 0; 0 0 0; t_checker(1:2) -1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
    in_pos = [0 0 0; 0 0 0; 0 0.2 -1.1; 0.5 0.5 -1.5; 0 -0.5 -1.5; 0 0 -1]';
    in_eul = [-0 0 0; 0 0 0; 0 0 0; 20 -20 0; -20 0 10; 0 0 0]';

    %elevated figure eight
    trajNames = 'elev8';
    in_times = [0.01, 0.3, ...
        3, 5, 7, 9, 11];
    in_pos = [0 0 -1; 0 0 -1; ...
        0.3 -0.5 -1; 0 -1 -1; 1 0.7 -1.5; -0.5 0.7 -1.3; 0 0 -1];

    simset.duration =in_times(end)+1;
    [wp_pos, ~, ~, ~, ~, ~, ~, wp_times] = minsnappolytraj(in_pos, in_times, (simset.duration*simset.simHz));
    wp_eul = minsnappolytraj(in_eul, in_times, (simset.duration*simset.simHz));
    
    trajOut = [wp_times; wp_pos; wp_eul]; 
    

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
            skip =20; %make the array smaller and smoother by skipping a few values
            wp_times = mc_times(1:skip:end,:); % [mc_times(1:5,:); mc_times(6:skip:end,:)];
            wp_pos =  mc_pos(:, 1:skip:end); %[mc_pos(:,1:5), mc_pos(:, 6:skip:end)];
            wp_eul =  mc_eul(:, 1:skip:end);%[mc_eul(:,1:5), mc_eul(:, 6:skip:end)];  
            wp_eul = rad2deg(unwrap(wp_eul, [], 2));
            wp_eul_smooth = smoothdata(wp_eul, 2,"rlowess", 0.05);
    
        
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

