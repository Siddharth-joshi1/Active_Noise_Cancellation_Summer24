
%% 
%              +-----------+                       +   
% x(k) ---+--->|   P(z)    |--yp(k)----------------> sum --+---> e(k)
%         |    +-----------+                          ^-   |
%         |                                           |    |
%% 
%% 
%         |        \                                ys(k)  |     
%         |    +-----------+          +-----------+   |    |
%         +--->|   C(z)    |--yw(k)-->|   S(z)    |---+    |
%         |    +-----------+          +-----------+        |
%         |            \                                   |
%         |             \----------------\                 |
%         |                               \                |
%         |    +-----------+          +-----------+        |
%         +--->|   Sh(z)   |--xs(k)-->|    LMS    |<-------+
%              +-----------+          +-----------+        
% 
% I used FIR filter to model P(z), C(z), S(z), and Sh(z).
% 
% Imagine that the noise x(k) is propagating from the source to the sensor,
% through the fluid medium P(z). The sensor measures the arriving noise as 
% yp(k). 
%
% To reduce noise, we generate another 'noise' yw(k) using the controller 
% C(z). We hope that it destructively interferes x(k). It means that the 
% controller has to be a model of the propagation medium P(z). Least mean 
% square algorithm is applied to adjust the controller coefficient/weight.
%
% However, there is also fluid medium S(z) that stay between the actuator 
% and sensor. We called it the secondary propagation path. So, to make the 
% solusion right, we need to compensate the adjustment process using Sh(z), 
% which is an estimate of S(z).
% 


% Set simulation duration (normalized) 
clear
T=100000; 

% We do not know P(z) and S(z) in reality. So we have to make dummy paths
P=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=P;

% Remember that the first task is to estimate S(z). So, we can generate a
% white noise signal,
noise_variance = 0.1;
x_iden= 1 * sqrt(noise_variance) * randn(T, 1);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);     % the state of Sh(z)
Shw=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.01;                         % learning rate
for k=1:T                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

% % Lets check the result
% subplot(2,1,1)
% plot([1:T], e_iden)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Identification error');
% subplot(2,1,2)
% stem(Sw) 
% hold on 
% stem(Shw, 'r*')
% ylabel('Amplitude');
% xlabel('Numbering of filter tap');
% legend('Coefficients of S(z)', 'Coefficients of Sh(z)')
% 
% 
% % The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% the noise: 
% X=randn(1,T);
noise_variance = 0.1;
X= 10 * sqrt(noise_variance) * randn(1, T);
% and measure the arriving noise at the sensor position,
Yd=filter(P, 1, X);
a=0*sin(2*pi*0.01*(1:T)');
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Cw=zeros(1,16);       % the weight of C(z)
Sx=zeros(size(Sw));   % the dummy state for the secondary path
e_cont=zeros(1,T); 
e2=zeros(1,T);% data buffer for the control error
Xhx=zeros(1,16);      % the state of the filtered x(k)
ahat_buffer=zeros(1,7);
x1=zeros(1,T);
% and apply the FxLMS algorithm
delta=0.0000001;
mu=0.000001;
out=zeros(1,T);% learning rate
for k = 1:T
    % Update secondary path based on the sample index
    if k <= 25000
        Sw =[0.1 0.25 0.05 0.21 0.05 0.025 0.001] ;
    elseif k <= 50000
        Sw =  1.25*P;
    elseif k <= 75000
        Sw =  P*0.75;
    else
        Sw = P*2;
    
    end
    % Cw = Cw./(sum(abs(Cw))+delta);% discrete time k
    Cx=[X(k) Cx(1:15)];            % update the controller state    
    Cy=sum(Cx.*Cw); 
    Cy1=Cy+a(k);
    % Cy1=Cy;% calculate the controller output	
    Sx=[Cy1 Sx(1:length(Sx)-1)]; 
    ahat_buffer=[a(k) ahat_buffer(1:size(Sw)-1)];% propagate to secondary path
    % x1(k)=sum(Sx.*Sw);

    e=Yd(k)-sum(Sx.*Sw);%output
    e1=e+sum(ahat_buffer.*Shw);
    e_cont(k)=e1;
    out(k)=e;
    % measure the residue
    Shx=[X(k) Shx(1:15)];          % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx; 
    % mse1(k) = mean(e_cont(1:k).^2);
    % e2(k)=e1;% adjust the controller weight
end
% 
figure
% subplot(3, 1, 1)
plot([1:T], X,'r')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Input Noise')
hold on
plot([1:T], e_cont,'b')
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Noise residue')

% xlim([ T]);  % Set x-axis limit to start from 500
% 
% subplot(3, 1, 2)
% plot([1:T], a)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Desired signal')
% % xlim([10000 T]);  % Set x-axis limit to start from 500

% subplot(3, 1, 3)
% plot([1:T], out)
% ylabel('Amplitude');
% xlabel('Discrete time k');
% legend('Output')
% xlim([10000 T]);  % Set x-axis limit to start from 500


