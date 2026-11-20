function bias = calcConstBias(idealImu, measImu, scale)

            scaledImu = scale*measImu;
            diffImu = scaledImu-idealImu;
            bias = mean(diffImu,2);
   
end