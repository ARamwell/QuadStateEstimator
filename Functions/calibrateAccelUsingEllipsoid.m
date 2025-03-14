%Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

% Import the data
%filepath = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_20250310_1058.csv';
data_imu = readmatrix(filepath); % Use readtable(filepath) if the CSV has headers

% Isolate accelerometer values
x = data_imu(:, 4);
y = data_imu(:, 5);
z = data_imu(:, 6);
accel_uncalib = [x, y, z];

%Plot uncalibrated values
figure(1)
plot3(x(:),y(:),z(:),"LineStyle","none","Marker","X","MarkerSize",8)

% Fit ellipsoid to data
[ center, radii, evecs, v, chi2 ] = ellipsoid_fit( [ x y z ], '' );

%draw fit
mind = min( [ x y z ] );
maxd = max( [ x y z ] );
nsteps = 50;
step = ( maxd - mind ) / nsteps;
[ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );

Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
          2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
          2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
p = patch( isosurface( x, y, z, Ellipsoid, -v(10) ) );
hold off;
set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
%view( -70, 40 );
axis vis3d equal;
camlight;
lighting phong;

% Calculate IMU correction terms
%g = 9.81;
a = evecs(:,1);
A = v(1);
B = v(2);
C = v(3);
D = v(4);
E = v(5);
F = v(6);
G = v(7);
H = v(8);
I = v(9);

S = [A D E; D B F; E F C];
eigen = eig(S);

gain = norm(g)/norm(a) * evecs *  diag([sqrt(eigen(1)), sqrt(eigen(2)), sqrt(eigen(3))]) * (evecs') ;

bias = center;

for row=1:size(accel_uncalib, 1)
    x1 = (accel_uncalib(row, :))' - bias;
    x2 = gain * x1;
    accel_calib(row, :) = x2'; 
end

%Output to .mat file
gainMatrix = gain;
biasSeparate = gain*bias;

accelCalibrationStruct= struct('gain', gainMatrix, 'bias', biasSeparate);

%save to .mat filefullfile('.', '/QuadSimEnv/Results/Traj-0005/fps_20');
save(fullfile('.', '/Resources/accelerometerCalibration'), "accelCalibrationStruct");

% Ask user if they want to save the file
choice = input('Do you want to save the data? (y/n): ', 's');

if lower(choice) == 'y'
            
    filename = input('Enter filename (without .mat extension): ', 's');
    if isempty(filename)
        filename = 'accelerometerCalibration'; % Default name if none provided
    end

    filepath = fullfile('.', '/Resources/', filename); % Default to current folder

    save(filepath, 'accelCalibrationStruct');
    fprintf('Data saved as %s\n', filepath);
else
    fprintf('Data not saved.\n');
end

