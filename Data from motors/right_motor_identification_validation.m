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
%% validation ?? not working
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
%% ?? not working
load("filtered_signal_right255_varianta2.mat")
signal_val = SpeedRight;
plot(signal_val), hold on, plot(y_val)
