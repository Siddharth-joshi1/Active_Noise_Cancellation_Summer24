clear
clear all
clc
T=100000; 
% filename = 'anc_test_audio.wav'; % File name of the audio
% info = audioinfo(filename); % Get audio file information
% T = info.TotalSamples;
P=[0.1 0.25 0.15 0.1 0.25 0.25 0.01]; %primary filter
s=P;

% s(1)=1;
% s(1)=1;
delta=0.0000001;
% Sw = Sw./(sum(abs(Sw))+delta);
% Pw = Pw./(sum(abs(Pw))+delta);

%first identifying shat which is the estimation of the secondary path

%reference filter will be Sw, here we assume the weights are not there that is w(n)=0  
% y_iden=filter(s, 1, x_iden);
% Then, start the identification process
Shat_buffer=zeros(1,16);     % the state of Sh(z)
Shat_w=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error
% and apply least mean square algorithm
                        % learning rate
% for k=1:T,                      % discrete time k
%     Shx=[x_iden(k) Shx(1:15)];  % update the state
%     Shy=sum(Shx.*Shat_w);	        % calculate output of Sh(z)
%     e_iden(k)=y_iden(k)-Shy;    % calculate error         
%     Shat_w=Shat_w+mu*e_iden(k)*Shx;   % adjust the weight
% end

noise_variance = 0.1;
x_iden= 0.0001 * randn(T, 1);
% % The second task is the active control. 
% X=randn(1,T);
% noise_variance = 0.1;
X= 10* sqrt(noise_variance) * randn(T, 1);



%primary path
d=filter(P, 1, X);

%desired audio
a=0*sin(2*pi*0.01*(1:T)');
% a = audioread(filename);
% a=a';
x_buffer=zeros(1,16);       
w=zeros(1,16);       
yhat_buffer=zeros(size(s));  
e_cont=zeros(1,T); 
e2=zeros(1,T);
Xhat=zeros(1,16);     
ahat_buffer=zeros(1,16);
x1=zeros(1,T);
% and apply the FxLMS algorithm
mu_shat=0.0001; 
mu=1.0;
out=zeros(1,T);% learning rate
x_iden_buffer=zeros(length(Shat_w));
for k = 1:T
    % Update secondary path based on the sample index
    if k <= 2500
        Sw =[0.1 0.25 0.05 0.21 0.05 0.025 0.001] ;
    elseif k <= 5000
        Sw =  1.25*P;
    elseif k <= 7500
        Sw =  P*0.75;
    else
        Sw = P*2;
    
    end
    x_buffer=[X(k) x_buffer(1:15)];            % update the controller state    
    y=sum(x_buffer.*w); %anti noise
    % propagate to secondary path
    % aa=sum(ahat_buffer.*inverse_filter);

    yhat=y+x_iden(k);

    % Cy1=Cy;% calculate the controller output	
    yhat_buffer=[yhat yhat_buffer(1:length(yhat_buffer)-1)]; 
    
    x_iden_buffer=[x_iden(k) x_iden_buffer(1:length(x_iden_buffer)-1)]; 
    x_iden_shat=sum(x_iden_buffer.*Shat_w);
   
    e=d(k)-sum(yhat_buffer.*s);%output
    ahat_buffer=[a(k) ahat_buffer(1:15)];
    
    e_cont(k)=e;
    out(k)=e;
    e2=e_cont(k)-x_iden_shat;
    Shat_buffer=[X(k) Shat_buffer(1:15)];          % update the state of Sh(z)
    Xhat=[sum(Shat_buffer.*Shat_w) Xhat(1:15)]; % calculate the filtered x(k)
    w=w+mu*e_cont(k)*Xhat; %LMS is used
    Shat_w=Shat_w+mu_shat*e2*x_iden_buffer;
    % e2(k)=e1;% adjust the controller weight
end
% info.SampleRate=T;
% % audiowrite('audio1_ANC.wav',e,Fs);
% % % Play noisy input signal
% soundsc(x_plus_a, info.SampleRate);
% 
% 
% % Pause for a few seconds before playing the output signal
% pause(length(out) / info.SampleRate);
% 
% 
% soundsc(a', info.SampleRate);
% % % Pause for a few seconds before playing the output signal
% pause(length(x_plus_a) / info.SampleRate);
% 
% % Play output signal
% soundsc(out, info.SampleRate);
% % Report the result
fs=16000;
audiowrite('reference2.wav', a, fs);
audiowrite('degraded2.wav', out, fs);
figure
plot([1:T], X,'r')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Input Noise')
hold on
plot([1:T], e_cont,'b')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')

% subplot(3,1,2)
% plot([1:T], a)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('desired signal')
% 
% subplot(3,1,3)
% plot([1:T],out)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('output')
