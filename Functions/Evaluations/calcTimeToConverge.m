function [tConv, idxConv, errOut] = calcTimeToConverge(t, absTrajErr, ateThreshold, areThreshold, holdTime)
%CALCTIMETOCONVERGE Find first pose-convergence time from state histories.
%
%   Convergence is defined as the first sample from which both ATE and ARE
%   remain below their thresholds for at least holdTime seconds.
%
%   Inputs:
%       t            - 1xN or Nx1 time vector, in seconds
%       trajErr      - trajectory error metrics
%       ateThreshold - translation threshold, in metres
%       areThreshold - rotation threshold, in degrees
%       holdTime     - required time below thresholds, in seconds
%
%   Outputs:
%       tConv        - convergence time in seconds, NaN if not converged
%       idxConv      - convergence index, NaN if not converged
%       errOut       - struct with fields ATE and ARE

    t = t(:)';

    tConv = NaN;
    idxConv = NaN;
    errOut = struct('ATE', [], 'ARE', []);

    if isempty(t) || isempty(absTrajErr)
        return;
    end

    n = min([numel(t), size(absTrajErr,2)]);
    t = t(1:n);
  
    valid = ~isnan(t) & all(~isnan(absTrajErr(1:2,:)),1);
    t = t(valid);
    absTrajErr = absTrajErr(:,valid);

    if isempty(t)
        return;
    end

    % Translation error
    ate = absTrajErr(2,:);
    are = absTrajErr(1,:);
    
    % posErr = xEst(1:3,:) - xTrue(1:3,:);
    % ate = vecnorm(posErr, 2, 1);
    % 
    % % Rotation error
    % qEst = xEst(4:7,:);
    % qTrue = xTrue(4:7,:);
    % 
    % qEst = qEst ./ vecnorm(qEst, 2, 1);
    % qTrue = qTrue ./ vecnorm(qTrue, 2, 1);
    % 
    % qDot = abs(sum(qEst .* qTrue, 1));
    % qDot = min(max(qDot, -1), 1);
    % are = 2 * acosd(qDot);
    % 
    % errOut.ATE = ate;
    % errOut.ARE = are;

    below = (ate <= ateThreshold) & (are <= areThreshold);

    for k = 1:numel(t)
        if ~below(k)
            continue;
        end

        idxEnd = find(t >= t(k) + holdTime, 1, "first");

        if isempty(idxEnd)
            idxEnd = numel(t);
        end

        if all(below(k:idxEnd))
            tConv = t(k);
            idxConv = k;
            return;
        end
    end
end