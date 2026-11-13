function [ekfResult] = analyseEkf_2(ekfResult, type)
%ANALYSEEKF_2 Summary of this function goes here
%   Detailed explanation goes here

%% Extract useful variables
numStates = size(ekfResult.P,1);
numTimes = size(ekfResult.elapsedTime,2);
size_groundTruth_nomState = size(ekfResult.trueState, 1);
if type == "es"
    size_groundTruth_errState = size(ekfResult.trueErrState,1);
end

%% trajectory tracking error

ekfResult.eval.trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');

%% simple error
ekfResult.eval.simpleErr.nomState = abs(ekfResult.trueState - ekfResult.x_(1:size_groundTruth_nomState,:));
if type =="es"
    ekfResult.eval.simpleErr.errState = abs(ekfResult.trueErrState - ekfResult.dx_(1:size_groundTruth_errState,:));
end

%% nees 
if type == "es"
    idx_v = 7; %index of first velocity state
    ekfResult.eval.nees.overall = evalNEES(ekfResult.dx_, ekfResult.PHat, ekfResult.trueErrState);
    ekfResult.eval.nees.orient =  evalNEES(ekfResult.dx_(4:idx_v-1,:), ekfResult.PHat(4:idx_v-1,4:idx_v-1,:), ekfResult.trueErrState(4:idx_v-1, :));
    ekfResult.eval.nees.pos =  evalNEES(ekfResult.dx_(1:3,:), ekfResult.PHat(1:3,1:3,:), ekfResult.trueErrState(1:3, :));
    ekfResult.eval.nees.vel =  evalNEES(ekfResult.dx_(idx_v:idx_v+2,:), ekfResult.PHat(idx_v:idx_v+2, idx_v:idx_v+2,:), ekfResult.trueErrState(idx_v:idx_v+2, :));
else
    idx_v = 8;
    ekfResult.eval.nees.overall = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);
    ekfResult.eval.nees.orient =  evalNEES(ekfResult.x_(4:idx_v-1,:), ekfResult.PHat(4:idx_v-1,4:idx_v-1,:), ekfResult.trueState(4:idx_v-1, :));
    ekfResult.eval.nees.pos =  evalNEES(ekfResult.x_(1:3,:), ekfResult.PHat(1:3,1:3,:), ekfResult.trueState(1:3, :));
    ekfResult.eval.nees.vel =  evalNEES(ekfResult.x_(idx_v:idx_v+2,:), ekfResult.PHat(idx_v:idx_v+2, idx_v:idx_v+2,:), ekfResult.trueState(idx_v:idx_v+2, :));
end


%% nis
ekfResult.eval.nis.overall = evalNIS(ekfResult.y, ekfResult.S);
ekfResult.eval.nis.pos = evalNIS(ekfResult.y(1:3,:), ekfResult.S(1:3,1:3,:));
if type =="es"
    ekfResult.eval.nis.orient= evalNIS(ekfResult.y(4:idx_v-1,:), ekfResult.S(4:idx_v-1, 4:idx_v-1 ,:));
else
    ekfResult.eval.nis.orient= evalNIS_noq(ekfResult.y(4:idx_v-1,:), ekfResult.S(4:idx_v-1, 4:idx_v-1 ,:));
end

%% time-to-converge?
 [ekfResult.eval.ttc, idxTtc, ~]=calcTimeToConverge(ekfResult.elapsedTime, ekfResult.trajErr.AbsoluteError', 0.3,5, 1); %ate and are below threshold for certain time


%% 1-sigma nominal state bounds
%separate nominal state and error state, have channel specific version, and
%also have type specific version for good comparison
ekfResult.eval.uncertainty.channel = createArray(numStates, numTimes);
for t = 1:numTimes
    ekfResult.eval.uncertainty.channel(:,t) = sqrt(abs(diag(ekfResult.P(:,:,t))));

    %calculate rotation separately
    if type == "es"
        Ptheta = ekfResult.P(4:6,4:6,t);
    else %convert to a world-frame angular uncertainty (requires current state est)
        q = ekfResult.x_(4:7, t);
        J = 0.5 * [-q(2:4)';
                     q(1)*eye(3) - skewsym(q(2:4))];
        Ptheta = 16 * J' * ekfResult.P(4:7,4:7,t) * J;
    end 
    ekfResult.eval.uncertainty.rot.raw(:,t) = sqrt(abs(diag(Ptheta)));
    ekfResult.eval.uncertainty.rot.bound95(t) = (sqrt(7.815 * max(eig(Ptheta)))); %for the 95% confidence bounds of the longest radius of the uncertaintyellipsoid
    ekfResult.eval.uncertainty.rot.RMS(t) = 2*trace(Ptheta); 
end


end

