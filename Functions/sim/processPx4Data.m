
function [ekfResult] = processPx4Data(srcFile, aidingActive, groundTruth)
    
    if isempty(srcFile)
        ekfResult = 0;
    else
       
        ulog = ulogreader(srcFile);
        
        msg_estStates = readTopicMsgs(ulog, 'TopicNames', 'estimator_states');
        msg_sense = readTopicMsgs(ulog, 'TopicNames', 'sensor_combined');
    
        ekfResult.elapsedTime = seconds(msg_estStates.TopicMessages{1,1}.timestamp_sample)';
        ekfResult.time = ekfResult.elapsedTime;% msg_estStates.TopicMessages{1,1}.timestamp_sample';
        %ekfResult.time.Format = 'dd:hh:mm:';
    
        
        p = msg_estStates.TopicMessages{1,1}.states(:,8:10)';
        q = msg_estStates.TopicMessages{1,1}.states(:,1:4)';
        v = msg_estStates.TopicMessages{1,1}.states(:,5:7)';
        bg = msg_estStates.TopicMessages{1,1}.states(:,11:13)';
        ba = msg_estStates.TopicMessages{1,1}.states(:,14:16)';
        % for t = 1:size(p, 2)
        %     p_offset(:,t) = p(:,t) + pose0(1:3);
        % end
        ekfResult.x_ = [p; q; v; bg; ba];
    
        P_p = msg_estStates.TopicMessages{1,1}.covariances(:,5:7)';
        P_q =msg_estStates.TopicMessages{1,1}.covariances(:,1:4)';
        P_v= msg_estStates.TopicMessages{1,1}.covariances(:,8:10)';
        P_bg= msg_estStates.TopicMessages{1,1}.covariances(:,11:13)';
        P_ba= msg_estStates.TopicMessages{1,1}.covariances(:,14:16)';
        for t = 1: size(P_p, 2)
            ekfResult.P(:, :, t) = diag([P_p(:,t); P_q(:,t); P_v(:,t); P_bg(:,t); P_ba(:,t)]');
        end
    
    
        % measurements
        if aidingActive == true
            msg_aidpos = readTopicMsgs(ulog, 'TopicNames', 'estimator_aid_src_ev_pos');
            msg_aidhgt = readTopicMsgs(ulog, 'TopicNames', 'estimator_aid_src_ev_hgt');
            %ekfResult.statePred = out.ekf_xHat.signals.values'; can't get this
            %ekfResult.measPred = out.ekf_zHat.signals.values'; %measurements map
            %directly
            xy = msg_aidpos.TopicMessages{1,1}.observation';
            z = msg_aidhgt.TopicMessages{1,1}.observation';
            %yaw?
            ekfResult.z =  [xy; z];
        
            %innovations
            S_xy = msg_aidpos.TopicMessages{1,1}.innovation_variance';
            S_z = msg_aidhgt.TopicMessages{1,1}.innovation_variance';
            for t = 1: size(S_z, 2)
                ekfResult.S(:, :, t) = diag([S_xy(:,t); S_z(:,t)]');
            end
            y_xy = msg_aidpos.TopicMessages{1,1}.innovation_variance';
            y_z= msg_aidhgt.TopicMessages{1,1}.innovation_variance';
            ekfResult.y = [y_xy; y_z];
        else
            ekfResult.z = nan(3, size(ekfResult.x_, 2));
            ekfResult.y = nan(3, size(ekfResult.x_, 2));
            ekfResult.S = nan(3, 3, size(ekfResult.x_, 2));
        end
    
        %sensor readings
        ekfResult.u = [msg_sense.TopicMessages{1,1}.gyro_rad'; msg_sense.TopicMessages{1,1}.accelerometer_m_s2'];
        
        %ekfResult.timeSinceLastCorrection= timeSinceLastCorrection;
    
        %ekfResult.zIn;
    
    
        %% add ground truth?
        if ~isempty(groundTruth)
            %get EKF time-aligned ground truth
            indices = selectClosestTimeIndices(ekfResult.time, groundTruth.quad.time);
            ekfResult.trueState =  groundTruth.quad.state(:,indices);
        
            %get measurement time-aligned ground truth
            if aidingActive == true
                aid_logical_px4 = ~isnan(ekfResult.z(1, :));
                aid_indices_px4 = find(aid_logical_px4);
                indices = selectClosestTimeIndices(ekfResult.time(:,aid_indices_px4), groundTruth.quad.time);
                ekfResult.truePoseAid = groundTruth.quad.state(1:7,indices);
            end
        end
    end
end