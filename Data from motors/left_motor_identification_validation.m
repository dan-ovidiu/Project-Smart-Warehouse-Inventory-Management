%% RAW DATA: left/right motor's RPM for PWM = 85;
clear
data = readtable("PWM_85.csv");

speedLeft85PWM = data.("Var1");
speedRight85PWM = data.("Var2");

% t = 0.01 : 0.01 : 1.96;
% plot(t,speedLeft85PWM), xlabel("time"),ylabel("RPM")
% title("RPM for left motor for 85 PWM value")
% 
% figure,plot(t,speedRight85PWM), xlabel("time"),ylabel("RPM")
% title("RPM for right motor for 85 PWM value")

%% filtered signals for PWM = 85
load("filtered_signal_left85.mat") % imported from PhyCharm
filtered_signal_left = signal;
load("filtered_signal_right85.mat") % imported from PhyCharm
filtered_signal_right = signal;
%%
close all
plot(time,speedLeft85PWM)
hold on
plot(time,filtered_signal_left)

figure
plot(time,speedRight85PWM)
hold on
plot(time,filtered_signal_right)
%% left motor model
u = zeros(1,196);
u(20:180) = 2.47;

u_id = u;
y_id = filtered_signal_left;
% plot(filtered_signal_left)

yss = 60;
uss = 2.47;

K = yss / uss;
t0 = 20;
t1 = 21;
T = (t1 - t0) / 100;

H = tf(K,[T,1]);

y_pred = lsim(H,u_id,time);

figure
plot(filtered_signal_left), hold on, plot(y_pred)

data_id = iddata(filtered_signal_left(:),u_id(:),0.01);
model = iddata(y_pred(:),u_id(:),0.01);
figure, compare(data_id,model), title("Training: Filtered identification data vs model for left motor at PWM = 85")
% K = 24.2915, T = 0.01 
%% validare 
load("filtered_signal_left255.mat")
signal_val = signal;
u_val = zeros(1,153);
u_val(9:145) = 7.4; 
% data_val = iddata(signal_val(:),u_val(:),0.01);
% compare(data_val,model)
% plot(signal_val)
y_val = lsim(H,u_val,time);
plot(signal_val), hold on, plot(y_val)
title("Validation: Filtered validation data vs model for left motor at PWM = 255")
%%
model_arx = arx(data_id,[1 1 0]);
model_iv = iv4(data_id,[1 1 0]);
data_val = iddata(signal_val(:),u_val(:),0.01);
compare(data_val,model_arx,model_iv)
