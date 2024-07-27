
clc;
clear;
close all;
% Parameters
% filename = 'anc_test_audio.wav'; % File name of the audio
% info = audioinfo(filename); % Get audio file information
% N = info.TotalSamples; % Length of signal
% Parameters
N = 10000; % Length of signal
M = 32; % Filter order
delta = 1e-6; % Small constant to avoid division by zero
noise_variance = 0.1; % Variance of the Gaussian noise

% Signals
x = 10 * sqrt(noise_variance) * randn(N, 1); % Gaussian noise signal
Fs=96000;
freq=1000;
% a = audioread(filename); % Load audio data
% a = 1* sin(2 * pi * (1:N) * (freq/Fs)); % Desired signal (zero amplitude)

% a=a';
a = 0*sin(2 * pi * 0.01 * (1:N)');
x_plus_a = a + x; % Desired signal with added noise

% Filters
p = ones(M, 1); % Primary path filter
p(1) = 1;
p(2) = 0.8;
p(3) = 1.3;

s = ones(M, 1); % Secondary path filter
s(1)=1.4;
s(2)=1.6;
s(3)=0.6;
% Normalize the secondary path filter

shat = ones(M,1); % Set estimated secondary path filter equal to the normalized s

% Initialize adaptive filter weights
w = ones(M, 1);


% Buffers for input signals
d_buffer = zeros(M, 1);

x_buffer = zeros(M, 1);
a_buffer = zeros(M, 1); % Buffer for the desired signal
y_buffer = zeros(M, 1); % Buffer for the output signal passed through s
yhat_buffer = zeros(M, 1);
e_buffer = zeros(M, 1);
% Initialize final error buffer
final_error_buffer = zeros(M, 1); % Buffer for storing the past 32 values of final_error

% Output signals
y = zeros(N, 1);
e = zeros(N, 1);
final_error = zeros(N, 1);

% Calculate average power of the input signals
P_x = mean(x.^2);
P_a = mean(a.^2);

% Determine step sizes based on the given criteria
mu_w = 0.01; % Step size for NLMS algorithm for w(n)
mu_shat = 0.01; % Step size for NLMS algorithm for shat(n)

% ANC System Implementation
e_arr = zeros(N, 1);
y_arr = zeros(N, 1);
f_error = zeros(N, 1);
s = s./(sum(abs(s))+delta);
p = p./(sum(abs(p))+delta);
w = w./(sum(abs(w))+delta);   
shat = shat./(sum(abs(shat))+delta);


for n = 1:N
    % Update input signal buffer
    
    
    x_buffer = [x(n); x_buffer(1:M-1)];
    
    w = w./(sum(abs(w))+delta);   
    
    % Update desired signal buffer
    a_buffer = [a(n); a_buffer(1:M-1)];
    
    % Primary path output
    d = p' * x_buffer;
    d_buffer = [d; d_buffer(1:M-1)];
    shat = shat./(sum(abs(shat))+delta);
    % Estimated secondary path output
    xhat = shat' * x_buffer;
   
    % Adaptive filter output (anti-noise)
    y = w' * x_buffer;
   
    % Output signal
    y_plus_a = a(n) + y;
   
    % Update y_buffer
    y_buffer = [y_plus_a; y_buffer(1:M-1)];
    
    % Output signal passed through the secondary path
    yhat = s' * y_buffer;
    yhat_buffer = [yhat; yhat_buffer(1:M-1)];
 
    % Error signal
    e = -d + yhat; % Final output after secondary path
  
    % Estimated desired signal passed through estimated secondary path
    ahat = shat' * a_buffer;
    % ahat = ahat/32;
    % Final error
    final_error = -ahat + e;
    
    % Update final error buffer
    final_error_buffer = [final_error; final_error_buffer(1:M-1)];
    
    % Normalize the Filter
    norm_factor_w = x_buffer' * x_buffer + delta;
    w = w + (mu_w / norm_factor_w) * (final_error_buffer.*x_buffer);
    
    e_buffer = [e; e_buffer(1:M-1)];
    
    % Update estimated secondary path filter weights for shat using NLMS
    norm_factor_shat = yhat_buffer' * yhat_buffer + delta;
    shat = shat + (mu_shat / norm_factor_shat) * (final_error_buffer.*yhat_buffer);
   
    % Store error and output signals
    e_arr(n) = e;
    y_arr(n) = y;
    f_error(n) = final_error;
    
    % Print values of e and final_error every 50 iterations
    if mod(n, 50) == 0
        disp(['Iteration ', num2str(n), ': d = ', num2str(d), ', yhat = ', num2str(yhat(1)), ...
              ', y = ', num2str(y), ', e(d-yhat) = ', num2str(e), ', final_error = ', num2str(final_error),', ahat= ', num2str(ahat),...
              ', x = ', num2str(x(n)),', a= ', num2str(a(n))]);
    end
end
% info.SampleRate=N;
% audiowrite('audio1_ANC.wav',e,Fs);


% % Play noisy input signal
% soundsc(a, info.SampleRate);
% 
% % Pause for a few seconds before playing the output signal
% pause(length(x_plus_a) / info.SampleRate);
% 
% soundsc(x_plus_a, info.SampleRate);
% 
% % Pause for a few seconds before playing the output signal
% pause(length(x_plus_a) / info.SampleRate);
% 
% % Play output signal
% soundsc(e_arr, info.SampleRate);

% Plot results
figure;
subplot(5, 1, 1);
plot(a);
title('Desired Signal');
xlabel('Sample Index');
ylabel('Amplitude');

subplot(5, 1, 2);
plot(x);
title('Noise Signal');
xlabel('Sample Index');
ylabel('Amplitude');

subplot(5, 1, 3);
plot(x_plus_a);
title('Desired Signal + Noise');
xlabel('Sample Index');
ylabel('Amplitude');

subplot(5, 1, 4);
plot(e_arr);
title('Output Signal');
xlabel('Sample Index');
ylabel('Amplitude');

subplot(5, 1, 5);
plot(f_error);
title('Final Error Signal');
xlabel('Sample Index');
ylabel('Amplitude');
