clear
clear all
clc
% filename ='1-MB-WAV.wav'; % File name of the audio
% info = audioinfo(filename); % Get audio file information
% T = info.TotalSamples; % Length of signal
T = 1024*8; 
P = [0.1 0.25 0.15 0.1 0.25 0.25 0.01]; % primary filter
s = [0.1 0.25 0.15 0.1 0.25 0.25 0.01];

delta = 0.0000001;

noise_variance = 0.1;
x_iden = 0.001 * randn(T, 1);

% The second task is the active control.
X = 10* sqrt(noise_variance) * randn(T, 1);
figure;
plot(10*log10(abs(fft(X))),'r');

% Primary path
d = filter(P, 1, X);

% Desired audio
a = 50 * sin(2*pi*0.01*(1:T)');
% a = audioread(filename); % Load audio data
% Set the length of Shat_w (try different lengths: 16, 32, 64, etc.)
shat_length = 32;
Shat_buffer = zeros(1, shat_length);     % the state of Sh(z)
Shat_w = zeros(1, shat_length);          % the weight of Sh(z)
e_iden = zeros(1, T);                    % data buffer for the identification error
s_length=7;
x_buffer = zeros(1, shat_length);       
w = zeros(1, shat_length);       
yhat_buffer = zeros(1,s_length);  
e_cont = zeros(1, T); 
e2 = zeros(1, T);
Xhat = zeros(1, shat_length);     
ahat_buffer = zeros(1, shat_length);

mu_shat = 0.0000005; 
mu = 0.000005;
out = zeros(1, T); % learning rate
x_iden_buffer = zeros(length(Shat_w));

% Subband parameters
num_subbands = 16; % Define number of subbands
filter_order = 128; % Filter order for analysis and synthesis filters

% Design prototype low-pass filter for filter bank
proto_filter = fir1(filter_order-1, 1/num_subbands);

% Create analysis and synthesis filter banks
analysis_filters = zeros(num_subbands, filter_order);
synthesis_filters = zeros(num_subbands, filter_order);

for i = 1:num_subbands
    analysis_filters(i, :) = proto_filter .* exp(-1j * 2 * pi * (i-1) / num_subbands * (0:filter_order-1));
    synthesis_filters(i, :) = proto_filter .* exp(1j * 2 * pi * (i-1) / num_subbands * (0:filter_order-1));
end

% Adaptive filters for each subband
subband_filters = zeros(num_subbands, shat_length);
a_inv=zeros(1,T);
Shat_inverse=zeros(1,shat_length);
for k = 1:T
    % Update secondary path based on the sample index
    if k <= 25000
        s = P;
    elseif k <= 50000
        s = 0.5 * P;
    elseif k <= 75000
        s = 0.75 * P;
    else
        s = 0.25 * P;
    
    end
    % if k <= 25000
    %     s = P;
    % elseif k <= 50000
    %     % s = [0.01 0.05 0.015 0.01 0.25 0.025 0.1];
    %     s=0.5*P;
    % elseif k <= 75000
    %     % s = [0.124 0.5 0.015 0.001 0.025 0.025 0.1];
    %     s=
    % else
    %     s = [0.001 0.025 0.015 0.01 0.025 0.025 1];
    % end

    x_buffer = [X(k) x_buffer(1:shat_length-1)];            % update the controller state    
    y = sum(x_buffer .* w); % anti noise
   
    % Construct the inverse filter
    if k > shat_length
        P = length(Shat_w);
        H = fft(s, P); % Frequency response of the actual filter
        Hi = 1 ./ (H+delta); % Inverse of the frequency response
        
        % Ensure Hi is conjugate symmetric
        Hi = (Hi + conj(flipud(Hi))) / 2;
        
        % Compute the inverse FFT and take the real part
        Shat_inverse = real(ifft(Hi)); % Inverse filter in time domain
     end
    
    ahat_buffer = [a(k) ahat_buffer(1:shat_length-1)];
    % propagate to secondary path
    a_inverse = sum(ahat_buffer .* Shat_inverse);
    yhat = y + a_inverse - x_iden(k);
    a_inv(k)=a_inverse;
    % calculate the controller output
    yhat_buffer = [yhat yhat_buffer(1:s_length-1)]; 
    
    x_iden_buffer = [x_iden(k) x_iden_buffer(1:length(x_iden_buffer)-1)]; 
    x_iden_shat = sum(x_iden_buffer .* Shat_w);
   
    e = d(k) - sum(yhat_buffer .* s); % output
   
    e1 = e +a(k);
    e_cont(k) = e1;
    out(k) = e;
    e2 = e_cont(k) - x_iden_shat;
    
    Shat_buffer = [X(k) Shat_buffer(1:shat_length-1)];          % update the state of Sh(z)
    Xhat = [sum(Shat_buffer .* Shat_w) Xhat(1:shat_length-1)];  % calculate the filtered x(k)
    
    % Subband processing
    e_subbands = zeros(1, num_subbands);
    x_subbands = zeros(1, num_subbands);
    
    for i = 1:num_subbands
        e_subband = filter(analysis_filters(i, :), 1, e_cont(max(1, k-filter_order+1):k));
        x_subband = filter(analysis_filters(i, :), 1, X(max(1, k-filter_order+1):k));
        
        e_subbands(i) = real(e_subband(end)); % Ensure real values
        x_subbands(i) = real(x_subband(end)); % Ensure real values
    end
    
    % Update adaptive filters in each subband
    for i = 1:num_subbands
        subband_filters(i, :) = subband_filters(i, :) + mu * e_subbands(i) * x_subbands(i);
    end
    
    % Combine subband outputs
    combined_output = 0;
    for i = 1:num_subbands
        combined_output = combined_output + filter(synthesis_filters(i, :), 1, subband_filters(i, :));
    end
    
    w = w + mu * combined_output(1); % LMS is used for the first sample of combined_output
    Shat_w = Shat_w + mu_shat * e2 * x_iden_buffer;
end
out = real(out);

info.SampleRate=T;



% % Play noisy input signal
% soundsc(a./(sum(abs(a))+delta), info.SampleRate);
% 
% % % Pause for a few seconds before playing the output signal
% pause(length(a) / info.SampleRate);
% % 
% % soundsc(x_plus_a, info.SampleRate);
% 
% % Pause for a few seconds before playing the output signal
% % pause(length(x_plus_a) / info.SampleRate);
% 
% % Play output signal
% soundsc(out./(sum(abs(out))+delta), info.SampleRate);

% Ensure output is real


% Save and plot the results
fs = 16000;
audiowrite('reference2.wav', a, fs);
audiowrite('degraded2.wav', out, fs);

figure
% subplot(3, 1, 1)
% plot([1:T], e_cont)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Noise residue')
% % xlim([ T]);  % Set x-axis limit to start from 500
%6
% subplot(2, 1, 1)
% plot([1:T], a)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Desired signal')
% xlim([10000 T]);  % Set x-axis limit to start from 500

% subplot(2, 1, 2)
plot([1:T], out/6)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Output')
% xlim([10000 T]);  % Set x-axis limit to start from 500
