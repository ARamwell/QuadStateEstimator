function [results, flags] = runP3P(results, time, imagePnts, worldPnts, K, imageSize, squareSize, inlierThreshold, rtRounding, varargin)

    %This function runs the p3p method (or methods) of choice and returns
    %the resulting Rt matrix and reprojection error of each method inside a
    %struct.

    %inputs: 
    %           imagePnts :     (n x 2) matrix (i.e. column vectors) of the
    %                           coordinates of detected points in the image
    %           worldPnts :     (n x 3) matrix (i.e. column vectors) of the
    %                           coordinates of the detected points in the
    %                           world frame. Columns match with imagePnts.
    %           varargin :      list of methods to use, as strings
    %                           separated by commas

    %outputs:
    %           results:        struct of Rt matrix results

    %% Initialisations
    %Define a list of valid method names
    validMethods = {'Gao', 'KneipA', 'KneipO', 'KneipN','Grunert', 'Matlab'}; %if you add to this, you must also add a switch case


    %Check if 'prevResults' has any results in it
    prevMethodNames = fieldnames(results);
    newMethod =false;
    
    %Initialise output struct
    %results = struct();

    %Initalise flag to track invalid methods
    invalidMethodsFlag = false;
    validMethodUsed = false;

    %% Main
    %Loop over the input string (varargin, names of methods to be run)
    for i = 1:length(varargin)

        methodName = varargin{i}; %get current method name

        if any(strcmp(methodName, validMethods)) %if methodName is valid
            % Valid method: call it and store the result in the struct
            
            switch methodName
                case 'Gao'
                    %results.(methodName).Rt = method1(currentInput);
                case 'KneipA'
                    Rt = p3pRun.KneipA(imagePnts, worldPnts, K, squareSize, inlierThreshold);
                case 'KneipN'
                    Rt = p3pRun.KneipN(imagePnts, worldPnts, K, squareSize, inlierThreshold);
                case 'KneipO'
                    Rt = p3pRun.KneipO(imagePnts, worldPnts, K, squareSize, inlierThreshold);
                case 'Grunert'
                    Rt = p3pRun.Grunert(imagePnts, worldPnts, K, squareSize, inlierThreshold);
                case 'Matlab'
                    Rt = p3pRun.MatlabPnP(K, imagePnts, worldPnts, imageSize, 0.1);
            end

            if ~any(strcmp(methodName, prevMethodNames))
                results.(methodName).Rt = [];
                results.(methodName).time = [];
                results.(methodName).pose = [];
                results.(methodName).error = [];
                newMethod = true;
            end 

            results.(methodName).Rt(1:3, 1:4, end+1) = round(Rt, rtRounding);
            results.(methodName).time(1,end+1) = time;
            results.(methodName).pose(1:6,end+1) = p3pFuncs.rtToPose(Rt, 'ZYX');
            validMethodUsed = true;  % At least one valid method was used

            %for some reason, an extra Rt of zeros is added on the first
            %iteration -remove these
            if newMethod
                results.(methodName).Rt = results.(methodName).Rt(:,:,2);
            end
        
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
