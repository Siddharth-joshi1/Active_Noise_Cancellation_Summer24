clear
clear all
clc
T = 100000; 
P = [0.1 0.25 0.15 0.1 0.25 0.25 0.01]; % primary filter
s = P;

delta = 0.0000001;

% First identifying Shat which is the estimation of the secondary path

% Initialize variables
Shat_buffer = zeros(1, 16);     % the state of Sh(z)
Shat_w = zeros(1, 16);          % the weight of Sh(z)
e_iden = zeros(1, T);           % data buffer for the identification error

noise_variance = 0.1;
x_iden = 0.1 * randn(T, 1);

% The second task is the active control. 
X = 10 * sqrt(noise_variance) * randn(T, 1);

% Primary path
d = filter(P, 1, X);

% Desired audio
a = 50 * sin(2 * pi * 0.01 * (1:T)');

x_buffer = zeros(1, 16);       
w = zeros(1, 16);       
yhat_buffer = zeros(size(s));  
e_cont = zeros(1, T); 
e2 = zeros(1, T);
Xhat = zeros(1, 16);     
ahat_buffer = zeros(1, 16);
x1 = zeros(1, T);

mu_shat = 0.0000005; 
mu = 0.000005;
out = zeros(1, T); % learning rate
x_iden_buffer = zeros(length(Shat_w));

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

    x_buffer = [X(k) x_buffer(1:15)];            % update the controller state    
    y = sum(x_buffer .* w); % anti noise

    yhat = y + a(k) - x_iden(k);

    % calculate the controller output	
    yhat_buffer = [yhat yhat_buffer(1:length(yhat_buffer)-1)]; 
    
    x_iden_buffer = [x_iden(k) x_iden_buffer(1:length(x_iden_buffer)-1)]; 
    x_iden_shat = sum(x_iden_buffer .* Shat_w);
   
    e = d(k) + sum(yhat_buffer .* s); % output
    ahat_buffer = [a(k) ahat_buffer(1:15)];
    e1 = e - sum(ahat_buffer .* Shat_w);
    e_cont(k) = e1;
    out(k) = e;
    e2 = e_cont(k) + x_iden_shat;
    Shat_buffer = [X(k) Shat_buffer(1:15)];          % update the state of Sh(z)
    Xhat = [sum(Shat_buffer .* Shat_w) Xhat(1:15)];  % calculate the filtered x(k)
    w = w + mu * e_cont(k) * Xhat; % LMS is used
    Shat_w = Shat_w + mu_shat * e2 * x_iden_buffer;
end

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
% 
% subplot(3, 1, 2)
% plot([1:T], a)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Desired signal')

% subplot(3, 1, 3)
plot([1:T], out)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Output')
