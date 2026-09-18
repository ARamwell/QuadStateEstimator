
ekfMetricsArr(end+1) = ekfMetrics;

% %make sure ekfMetricsArr is loaded with everything you need.
% %if necessary, append px4 results:
% px4_gb = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-05_12-05-08_allReal_px4\ekfMetrics_px4_biasGrav.mat");
% px4_ev = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-05_12-05-08_allReal_px4\ekfMetrics_px4.mat");
% px4_gb = px4_gb.ekfMetrics;
% px4_ev = px4_ev.ekfMetrics;
% ekfMetricsArr(end+1) = px4_ev;
% ekfMetricsArr(end+1) = px4_gb;


% ekfMetrics_a0_simTune = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0_simTune_scrub1s.mat");
% ekfMetrics_a0_inflatedQz = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0_inflatedQz_scrub1s.mat");
% ekfMetrics_a0_inflatedQ = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0_inflatedQ_scrub1s.mat");
% ekfMetrics_a0_inflatedQ_reprojOnly = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0_inflatedQ_reprojOnly_scrub1s.mat");
% ekfMetrics_a0_simTune_reprojOnly=load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0_simTune_reprojOnly_scrub1s.mat");
% 
% ekfMetrics_a099_simTune = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_simTune_scrub1s.mat");
% ekfMetrics_a099_inflatedQ = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_inflatedQ_scrub1s.mat");
% ekfMetrics_a099_inflatedQ_reprojOnly = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_inflatedQ_reprojOnly_scrub1s.mat");
% ekfMetrics_a099_simTune_reprojOnly = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_simTune_reprojOnly_scrub1s.mat");
% 
% ekfMetrics_a099_inflatedQz = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_inflatedQz_scrub1s.mat");
% ekfMetrics_a099_inflatedQz_reprojOnly = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-12_s3-6\ekfMetrics_16elRa0.99_inflatedQz_ReprojOnly_scrub1s.mat");

ekfMetrics_a0_simTune = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-m11_s3-6_trueDt\ekfMetrics_16Ra0_simTune_trueDt.mat");
ekfMetrics_a0_inflatedQz = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-m11_s3-6_trueDt\ekfMetrics_16Ra0_inflatedQz_trueDt.mat");
ekfMetrics_a099_inflatedQz = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-m11_s3-6_trueDt\ekfMetrics_16Ra0.99_inflatedQz_trueDt.mat");
ekfMetrics_a099_reprojOnly = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\nano\mainStateEst\analyses\anal_m8-m11_s3-6_trueDt\ekfMetrics_16Ra0.99_inflatedQz_reprojOnly_trueDt.mat");

ekfMetrics_a0_simTune.ekfMetrics.ekfType = "16e-R-a0-simTune";
ekfMetrics_a0_inflatedQz.ekfMetrics.ekfType="16e-R-a0-inflQz";
ekfMetrics_a099_inflatedQz.ekfMetrics.ekfType="16e-R-a0.99-inflQz";
ekfMetrics_a099_reprojOnly.ekfMetrics.ekfType="16e-R-a0.99-reproj";

ekfMetricsArr(1) = ekfMetrics_a0_simTune.ekfMetrics;
ekfMetricsArr(end+1) =ekfMetrics_a0_inflatedQz.ekfMetrics;
ekfMetricsArr(end+1) = ekfMetrics_a099_inflatedQz.ekfMetrics;
ekfMetricsArr(end+1) = ekfMetrics_a099_reprojOnly.ekfMetrics;
% 
% ekfMetricsArr(end+1) =ekfMetrics_a099_inflatedQz.ekfMetrics;
% %ekfMetricsArr(end+1) =ekfMetrics_a099_inflatedQ.ekfMetrics;
% %ekfMetricsArr(end+1) =ekfMetrics_a099_inflatedQ_reprojOnly.ekfMetrics;
% ekfMetricsArr(end+1) =ekfMetrics_a099_inflatedQz_reprojOnly.ekfMetrics;



%%
%boxData_are = createArray(0,1); %column array
clear dynArgs_are dynArgs_ate dynArgs_vel dynArgs_ba dynArgs_bg 
cut = 0; %how many readings to cut (to account for bad initialisation)


%prepare data for box plot 
for i=1:size(ekfMetricsArr,2)

    orientErr_i = ekfMetricsArr(i).trajErr(1,cut+1:end)'; 
    posErr_i = ekfMetricsArr(i).trajErr(2,cut+1:end)';
    %posErr_i =rmoutliers(posErr_i, 'mean');
    

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

%maxAre = max(dynArgs_are)
dynArgs_are{end+1} = 'TailBinPct'; % assign everything above 95th percentile to a single bin
dynArgs_are{end+1} = 95;
dynArgs_are{end+1} = 0.05;   % bin size (degrees)
dynArgs_are{end+1} = 'absolute rotation error (degrees)';

dynArgs_ate{end+1} = 'TailBinPct';
dynArgs_ate{end+1} = 95;
dynArgs_ate{end+1} = 0.001;   % bin size (m)
dynArgs_ate{end+1} = 'absolute translation error (m)';

dynArgs_vel{end+1} = 'TailBinPct';
dynArgs_vel{end+1} = 95;
dynArgs_vel{end+1} = 0.005;   % bin size (m/s)
dynArgs_vel{end+1} = 'absolute velocity error (m/s)';

if exist('dynArgs_ba', 'var')
    dynArgs_bg{end+1} = 'TailBinPct';
    dynArgs_bg{end+1} = 95;
    dynArgs_bg{end+1} = 1e-4;   % bin size (rad/s)
    dynArgs_bg{end+1} = 'absolute gyroscope bias error (rad/s)';

    dynArgs_ba{end+1} = 'TailBinPct';
    dynArgs_ba{end+1} = 95;
    dynArgs_ba{end+1} = 0.005;   % bin size (m/s^2)
    dynArgs_ba{end+1} = 'absolute accelerometer bias error (m/s^2)';
end


% %% Plot
plotViolin(dynArgs_are{:})

plotViolin(dynArgs_ate{:})
plotViolin(dynArgs_vel{:})
if exist('dynArgs_ba', 'var')
    plotViolin(dynArgs_ba{:})
    plotViolin(dynArgs_bg{:})
end