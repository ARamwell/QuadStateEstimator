

%make sure ekfMetricsArr is loaded with everything you need.
%if necessary, append px4 results:
%px4_gb = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-05_12-05-08_allReal_px4\ekfMetrics_px4_biasGrav.mat");
%ekfMetricsArr(end+1) = px4_gb;

%boxData_are = createArray(0,1); %column array
clear boxData_are boxData_ate boxData_groups
last = 0;
cut = 20; %how many readings to cut (to account for bad initialisation)


%prepare data for box plot
for i=1:size(ekfMetricsArr,2)

    orientErr_i = ekfMetricsArr(i).trajErr(1,cut+1:end)';
    posErr_i = ekfMetricsArr(i).trajErr(2,cut+1:end)';
    group_i = ekfMetricsArr(i).ekfType;
    dataSize = size(posErr_i, 1);
    
    if i == 1
        boxData_are= orientErr_i;
        boxData_ate = posErr_i;
        boxData_groups= repmat(group_i, dataSize, 1);
        violinData_groups= repmat(i, dataSize, 1);
    else
        newFinal = last +dataSize;
        boxData_are(last+1:newFinal, 1) = orientErr_i;
        boxData_ate(last+1:newFinal, 1) = posErr_i;
        boxData_groups(last+1:newFinal, 1) = repmat(group_i, dataSize, 1);
        violinData_groups(last+1:newFinal, 1) = repmat(i, dataSize, 1);
    end

    last = newFinal;
end

%plot
areBox = figure;
boxplot(boxData_are, boxData_groups, 'Whisker',3);

ateBox = figure;
boxplot(boxData_ate, boxData_groups, 'Whisker',3);

figure;
