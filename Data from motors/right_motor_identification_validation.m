clear
data = readtable("PWM_85.csv");

speedRight85PWM = data.("Var2");

load("filtered_signal_right85.mat") % imported from PhyCharm: identification signal
filtered_signal_right = signal;

plot(speedRight85PWM), hold on, plot(filtered_signal_right)
%% identificare pe treapta cu PWM = 85
close all
u = zeros(1,196);
u(16:177) = 2.47;

u_id = u;
y_id = filtered_signal_right;
data_id = iddata(y_id(:),u_id(:),0.01);
model_arx = arx(data_id,[1 1 0]);

yss = mean(y_id(30:173));
uss = 2.47;

K = yss / uss;
t0 = 16;
t1 = 22;
T = (t1 - t0) / 100;

H = tf(K,[T,1]); % K = 30.1339 & T = 0.06

y_pred = lsim(H,u_id,time);

% figure
% plot(filtered_signal_right), hold on, plot(y_pred)

data_id = iddata(filtered_signal_right(:),u_id(:),0.01);
model = iddata(y_pred(:),u_id(:),0.01);
figure, compare(data_id,model,model_arx), title("Training: Filtered identification data vs model for right motor at PWM = 85")
%% training on PWM 255 
clear
load("filtered_signal_left255.mat")
y_id = signal;
u_id = zeros(1,153);
u_id(5:148) = 7.4;

yss = mean(y_id(18:144));
uss = 7.4;
K = yss / uss;

t0 = 5;
t1 = 9;
T = (t1 - t0) / 100;

H = tf(K,[T,1]); % K = 22.1452, T = 0.04
y_pred = lsim(H,u_id,time);
plot(y_id), hold on, plot(y_pred)
% plot(y_id)
%% tuning the PID controller
clear
K = 22.1452;
T = 0.04;
H = tf(K,[T,1]);
Hc = pidtune(H,'PID');
Hd = series(Hc,H);
H0 = feedback(Hd,1);
step(H0)
%% validation 1 ?? not working
close all
load("filtered_signal_right255.mat")
signal_val = signal;
% plot(signal_val)
u_val = zeros(1,153);
u_val(1:150) = 7.4;

A = -1 / T;
B = K / T;
C = 1;
D = 0;
Hss = ss(A,B,C,D); % y0 = 128.354
y0 = 128.354;

y_val = lsim(Hss,u_val,time,y0);
plot(signal_val), hold on, plot(y_val)
title("Validation: Filtered validation data vs model for right motor at PWM = 255")
%% validation 2 ?? not working
load("filtered_signal_right255_varianta2.mat")
signal_val = SpeedRight;
plot(signal_val), hold on, plot(y_val)
%% validation 3 ?? working
load("filtered_signal_right128.mat")
signal_val = signal;
u_val = zeros(1,136);
u_val(1:136) = 3.71;

N = length(signal_val);
Ts = mean(diff(time));
time_corrected = (0:N-1) * Ts;

y_val = lsim(H,u_val,time_corrected);

plot(signal_val),hold on, plot(y_val)
title("Validation: Filtered validation data vs model for right motor at PWM = 128")
legend("real","prediction")
