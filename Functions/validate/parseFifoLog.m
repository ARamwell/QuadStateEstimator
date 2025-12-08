function [rawData, Fs, stats] = parseFifoLog(filepath)
%PARSEFIFOLOG Fast parser for PX4 *_fifo_*.csv IMU logs.
%
%   [rawData, Fs, stats] = parseFifoLog(filepath)
%
%   INPUT
%     filepath  - path to a single FIFO CSV log (accel or gyro).
%
%   OUTPUT
%     rawData   - Nx3 matrix of concatenated samples in SI units
%                 (columns correspond to x, y, z).
%     Fs        - average sample rate [Hz] computed from per-packet dt.
%     stats     - struct with basic sanity-check information:
%                   .pd_x, .pd_y, .pd_z  fitted normal distributions
%                   .avg_dt              average sample period [s]
%
%   The CSV is assumed to have at least the following columns:
%       - 'scale' : scale factor for raw integer samples
%       - 'dt'    : total time span of the packet [us]
%       - 'x*'    : one or more x-axis samples (x, x1, x2, ...)
%       - 'y*'    : one or more y-axis samples
%       - 'z*'    : one or more z-axis samples
%
%   Trailing unused FIFO entries should be NaN (or all zero, which are
%   removed earlier in the pipeline). This function:
%       1) loads the table once,
%       2) drops all-zero columns,
%       3) pre-allocates the full output based on the valid-sample count,
%       4) fills the output with a single pass over the packets.
%
%   This avoids repeated vector concatenations inside the main loop and
%   is significantly faster on large logs.

% Import the data table (let caller handle file selection if desired)
T = readtable(filepath);  % assumes header row is present

% Drop columns that are entirely zero
numericT = table2array(T);
zeroCols = all(numericT == 0, 1);
T(:, zeroCols) = [];

% Identify the x/y/z sample columns
varNames = T.Properties.VariableNames;
xMask = startsWith(varNames, 'x');
yMask = startsWith(varNames, 'y');
zMask = startsWith(varNames, 'z');

X = table2array(T(:, xMask));
Y = table2array(T(:, yMask));
Z = table2array(T(:, zMask));

% Number of valid samples in each row (assume trailing NaNs are invalid)
nValid = sum(~isnan(X), 2);

% Total number of valid samples across the log
totalSamples = sum(nValid);

% Pre-allocate outputs
ax = zeros(totalSamples, 1, 'double');
ay = zeros(totalSamples, 1, 'double');
az = zeros(totalSamples, 1, 'double');
all_dts = zeros(totalSamples, 1, 'double');

scale = T.scale;
dt_us = T.dt;   % microseconds per packet

% Single pass to fill outputs
idxStart = 1;
for k = 1:height(T)
    n = nValid(k);
    if n <= 0
        continue;
    end

    idxEnd = idxStart + n - 1;

    % Take only the valid portion for this packet
    xvals = X(k, 1:n);
    yvals = Y(k, 1:n);
    zvals = Z(k, 1:n);

    sc = double(scale(k));

    % Convert to SI units and store
    ax(idxStart:idxEnd) = double(xvals(:)) * sc;
    ay(idxStart:idxEnd) = double(yvals(:)) * sc;
    az(idxStart:idxEnd) = double(zvals(:)) * sc;

    % Per-sample dt (us / n, converted to seconds)
    dt_s = (double(dt_us(k)) / double(n)) * 1e-6;
    all_dts(idxStart:idxEnd) = dt_s;

    idxStart = idxEnd + 1;
end

% Truncate if any rows had nValid == 0
if idxStart <= totalSamples
    ax(idxStart:end) = [];
    ay(idxStart:end) = [];
    az(idxStart:end) = [];
    all_dts(idxStart:end) = [];
end

rawData = [ax ay az];

% Sample rate from average dt
avg_dt = mean(all_dts);
Fs = 1 / avg_dt;

% Basic sanity checks (normal fits)
pd_x = fitdist(ax, 'Normal');
pd_y = fitdist(ay, 'Normal');
pd_z = fitdist(az, 'Normal');

if nargout > 2
    stats = struct();
    stats.pd_x = pd_x;
    stats.pd_y = pd_y;
    stats.pd_z = pd_z;
    stats.avg_dt = avg_dt;
else
    stats = struct();
end

end


