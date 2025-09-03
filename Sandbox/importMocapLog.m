function [Rt_hist, pq_hist, time_hist, elapsedTime_hist] = importMocapLog(file, T_mocap2world, T_mq2rq)
%IMPORTMOCAPLOG Summary of this function goes here
%   Detailed explanation goes here

R_rq2mq = T_mq2rq(1:3, 1:3)';
t_rq2mq = - R_rq2mq * T_mq2rq(1:3, 4);
T_rq2mq = [ [R_rq2mq; 0 0 0] [t_rq2mq; 1] ];

mocapData = load(file);

numDataPoints = size(mocapData.timestamps_mocap, 1); 
Rt_hist = createArray(3,4,numDataPoints); %initialise output array
pq_hist = createArray(7, numDataPoints);
time_hist = datetime(mocapData.timestamps_mocap', 'Format', 'yyyyMMdd_HHmmss_SSS');
elapsedTime_hist = createArray(1, numDataPoints);

for i=1:numDataPoints

    q_mq2mw = mocapData.mocapMsgLog(i, 4:7); %read quaternion 
    R_mq2mw = quat2rotm(q_mq2mw);
    t_mq2mw = mocapData.mocapMsgLog(i, 1:3); %read position

    T_mq2mw = [[R_mq2mw; 0 0 0] [t_mq2mw'; 1]];

    T_mq2rw = T_mocap2world * T_mq2mw;
    
    T_rq2rw = T_mq2rw * T_rq2mq;

    Rt_hist(:,:,i) = T_rq2rw(1:3, 1:4);
    pq_hist(:,i) = [T_rq2rw(1:3, 4); rotm2quat(T_rq2rw(1:3, 1:3))'];

    if i==1
        elapsedTime_hist(:, i) = 0;
        
    else
        elapsedTime_hist(:, i) = milliseconds(time_hist(:, i) - time_hist(:, 1))/1000;
    end
end

end

