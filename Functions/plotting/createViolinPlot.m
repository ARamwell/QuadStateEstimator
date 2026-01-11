

% %make sure ekfMetricsArr is loaded with everything you need.
% %if necessary, append px4 results:
% px4_gb = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-05_12-05-08_allReal_px4\ekfMetrics_px4_biasGrav.mat");
% px4_ev = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-05_12-05-08_allReal_px4\ekfMetrics_px4.mat");
% px4_gb = px4_gb.ekfMetrics;
% px4_ev = px4_ev.ekfMetrics;
% ekfMetricsArr(end+1) = px4_ev;
% ekfMetricsArr(end+1) = px4_gb;

%boxData_are = createArray(0,1); %column array
clear dynArgs_are dynArgs_ate dynArgs_vel dynArgs_ba dynArgs_bg 
cut = 0; %how many readings to cut (to account for bad initialisation)


%prepare data for box plot 
for i=1:size(ekfMetricsArr,2)

    orientErr_i = ekfMetricsArr(i).trajErr(1,cut+1:end)'; 
    posErr_i = ekfMetricsArr(i).trajErr(2,cut+1:end)';
    velErr_i = abs(vecnorm(ekfMetricsArr(i).simpleErr(8:10,cut+1:end), 2, 1));
    velErr_i = rmoutliers(velErr_i, 'mean');


    group_i = ekfMetricsArr(i).ekfType;
    dataSize = size(posErr_i, 1);

       
    %if i == 1
    %    dynArgs_are = {orientErr_i, group_i};
    %    dynArgs_ate = {posErr_i, group_i};
    %else
        dynArgs_are{i*2-1} = orientErr_i;
        dynArgs_are{i*2} = group_i;
        dynArgs_ate{i*2-1} = posErr_i;
        dynArgs_ate{i*2} = group_i;
        dynArgs_vel{i*2-1} = velErr_i;
        dynArgs_vel{i*2} = group_i;

    %end

    if size(ekfMetricsArr(i).simpleErr, 1) > 10
        baErr_i = abs(vecnorm(ekfMetricsArr(i).simpleErr(11:13,cut+1:end), 2, 1));
        bgErr_i = abs(vecnorm(ekfMetricsArr(i).simpleErr(14:16,cut+1:end), 2, 1));
        if ~exist('dynArgs_ba', 'var')
            dynArgs_ba{1} = baErr_i;
            dynArgs_bg{1} = bgErr_i;
        else
            dynArgs_ba{end+1} = baErr_i;
            dynArgs_bg{end+1} = bgErr_i;
        end
        dynArgs_bg{end+1} = group_i;
        dynArgs_ba{end+1} = group_i;
    end


end

dynArgs_are{end+1} = 150;
dynArgs_are{end+1} = 'absolute rotation error (degrees)';

dynArgs_ate{end+1} = 150;
dynArgs_ate{end+1} = 'absolute translation error (m)';

dynArgs_vel{end+1} = 150;
dynArgs_vel{end+1} = 'absolute velocity error (m/s)';

if exist('dynArgs_ba', 'var')
    dynArgs_bg{end+1} = 150;
    dynArgs_bg{end+1} = 'absolute gyroscope bias error (rad/s)';

    dynArgs_ba{end+1} = 150;
    dynArgs_ba{end+1} = 'absolute accelerometer bias error (m/s^2)';
end


%% Plot
plotViolin(dynArgs_are{:})

plotViolin(dynArgs_ate{:})

plotViolin(dynArgs_vel{:})

if exist('dynArgs_ba', 'var')
    plotViolin(dynArgs_ba{:})

    plotViolin(dynArgs_bg{:})
end