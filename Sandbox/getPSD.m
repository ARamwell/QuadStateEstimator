x = rawAccel_stable(:,1);
fs = 8000;
t=0:1/fs:(size(x,1)-1)/fs;

N = length(x);
xdft = fft(x);
xdft = xdft(1:N/2+1);
psdx = (1/(fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:fs/length(x):fs/2;

figure;
plot(freq,pow2db(psdx));
grid on;
title("Periodogram Using FFT");
xlabel("Frequency (Hz)");
ylabel("Power/Frequency (dB/Hz)");

Y = fft(x);
figure;
plot(fs/N*(0:N-1),abs(Y),"LineWidth",3)
title("Complex Magnitude of fft Spectrum")
xlabel("f (Hz)")
ylabel("|fft(X)|")