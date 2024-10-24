function plots = initPlots(varargin)

    %Define a list of valid method names
    validMethods = {'Gao', 'KneipA', 'KneipO', 'KneipN','Grunert', 'Matlab', 'Verification', 'EKF'}; %if you add to this, you must also add a switch case
    
    %Initialise output struct
    plots = struct();

    %Initalise flag to track invalid methods
    invalidMethodsFlag = false;
    validMethodUsed = false;
    

    %% MAIN
    close all

    %Initialise trajectory plot
    trajFig = figure();
    worldAx = p3pPlotting.initPlot(trajFig, [-1000,1000], [-1000,1000], [0 2000]);
    
    %Add figure and axes to struct
    plots.traj.Type = 'trajectory';
    plots.traj.Fig = trajFig;
    plots.traj.Ax = worldAx;

    %Add trajectories based on varargin
    %Loop over the input string (varargin, names of methods to be run)
    for i = 1:length(varargin)

        methodName = varargin{i}; %get current method name

        if any(strcmp(methodName, validMethods)) %if methodName is valid
            % Valid method: call it and store the result in the struct
            
            switch methodName
                case 'Gao'
                    %results.(methodName).Rt = method1(currentInput);
                case 'KneipA'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-or', '-r');
                case 'KneipN'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-om', '-m');
                case 'KneipO'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-ok', '-k');
                case 'Grunert'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-oc', '-c');
                case 'Matlab'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-ob', '-b');
                case 'Verification'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, methodName, '-og', '-g');
                case 'EKF'
                    [lineObj, axTextArr, axLineArr] = p3pPlotting.addTraj(plots.traj.Ax, 'EKF', '-or', '-r');

            end
            
            %Add plot to plot struct
            plots.traj.plotLines.(methodName).line = lineObj;
            plots.traj.plotLines.(methodName).frameText = axTextArr;
            plots.traj.plotLines.(methodName).frameLines = axLineArr;
            plots.traj.plotLines.(methodName).Data =[];
            
            validMethodUsed = true;  % At least one valid method was used
        
        else
            % Invalid method: set the flag and issue a warning
            warning('Unknown method: %s. Skipping this method.', methodName);
            invalidMethodsFlag = true;
        end
    end
    
    % If no valid methods were used, show a message
    if ~validMethodUsed
        warning('No valid methods were used.');
    end






end