% ---------------------------------------------------------------------------------------------
% Turtlebot 2: encoders info plot and analysis
% Copyright (C) 2015 Ana Cruz-Martín, Juan Antonio Fernández-Madrigal (anacruzmartin@gmail.com)
% 
% Measurements are read from rows of a file with this format:
% [msgs_secs msgs_nsecs sensor_timestamp left_enc right_enc]
% where
% msgs_secs = secs of the ROS message timestamp
% msgs_nsecs = nsecs of the ROS message timestamp
% sensor_timestamp = sensor measurement timestamp (msecs)
% left_enc = left encoder reading
% rigth_enc = right encoder reading
% ---------------------------------------------------------------------------------------------


max_value_encoder = 65535;  % max measurement taken by the encoder
sampling_period = 0.02;   % time gap between sensor readings (secs)
ticks2rads = (2*pi)/2578.33;  % encoder: 2578.33 ticks/wheel revolution (see Kobuki docs)

load('datafile.txt');   % file where measurements are saved


% We plot the measurements taken by the encoders
figure;
t = datafile(:,3)/1e3;
plot(t,datafile(:,4),'.-b');
hold on;
plot(t,datafile(:,5),'.-r');
title('Turtlebot 2 encoders measurements');
xlabel('Sensor timestamp (secs)');
ylabel('Encoder measurement (ticks)');
legend('Left encoder','Right encoder');

figure;
t = datafile(:,3)/1e3;
t = t-t(1);
plot(t,datafile(:,4),'.-b');
hold on;
plot(t,datafile(:,5),'.-r');
title('Turtlebot 2 encoders measurements');
xlabel('Absolute sensor timestamp (secs)');
ylabel('Encoder measurement (ticks)');
legend('Left encoder','Right encoder');

figure;
t = datafile(:,1)+(datafile(:,2)/1e9);
plot(t,datafile(:,4),'.-b');
hold on;
plot(t,datafile(:,5),'.-r');
title('Turtlebot 2 encoders measurements');
xlabel('Message timestamp (secs)');
ylabel('Encoder measurement (ticks)');
legend('Left encoder','Right encoder');

figure;
t = datafile(:,1)+(datafile(:,2)/1e9);
t = t-t(1);
plot(t,datafile(:,4),'.-b');
hold on;
plot(t,datafile(:,5),'.-r');
title('Turtlebot 2 encoders measurements');
xlabel('Absolute message timestamp (secs)');
ylabel('Encoder measurement (ticks)');
legend('Left encoder','Right encoder');


% We check if encoders reset through the experiment
% If encoders reset, we readjust values adding the max value stored by the encoder, for further plots
left_encoder_row = find(abs(diff(datafile(:,4))) > 1000);
right_encoder_row = find(abs(diff(datafile(:,5))) > 1000);

if (left_encoder_row) datafile((left_encoder_row+1):end,4) = datafile((left_encoder_row+1):end,4)+(max_value_encoder+1); end;
if (right_encoder_row) datafile((right_encoder_row+1):end,5) = datafile((right_encoder_row+1):end,5)+(max_value_encoder+1); end;   


% We plot the speed of both wheel s
% (speed = variation of encoders reading divided by time)
figure;
t = (datafile(2:end,3))/1e3;
plot(t,diff(datafile(:,4)*ticks2rads)/sampling_period,'b.-');
hold on;
plot(t,diff(datafile(:,5)*ticks2rads)/sampling_period,'r.-');
title('Turtlebot 2 wheels speeds');
xlabel('Sensor timestamp (secs)');
ylabel('Wheel speed (rads/sec)');
legend('Left encoder','Right encoder');

figure;
t = (datafile(2:end,3))/1e3;
t = t-t(1);
plot(t,diff(datafile(:,4)*ticks2rads)/sampling_period,'b.-');
hold on;
plot(t,diff(datafile(:,5)*ticks2rads)/sampling_period,'r.-');
title('Turtlebot 2 wheels speeds');
xlabel('Absolute sensor timestamp (secs)');
ylabel('Wheel speed (rads/sec)');
legend('Left encoder','Right encoder');


% Motors: identification with a first order system
figure;
t = (datafile(2:end,3))/1e3;
t = t-t(1);
plot(t,diff(datafile(:,4)*ticks2rads)/sampling_period,'b.-');
hold on;
left_encoder_data = diff(datafile(:,4)*ticks2rads)/sampling_period;
left_encoder_length = length(left_encoder_data);
K = mean(left_encoder_data((left_encoder_length-100):left_encoder_length));
tau63_index = max(find(left_encoder_data < K*.63));
tau28_index = max(find(left_encoder_data < K*.28));
tau63 = t(tau63_index);
tau28 = t(tau28_index);
tau = 3/2*(tau63-tau28);
tau = tau/2;
motorsis = tf(K/tau,[1 1/tau]);
[a,motort] = step(motorsis,t);
plot(motort,a,'k--');
title('Turtlebot 2 left motor identification');
xlabel('Absolute sensor timestamp (secs)');
ylabel('Wheel speed (rads/sec)');
legend('Left encoder','First order system Identification');

figure;
t = (datafile(2:end,3))/1e3;
t = t-t(1);
plot(t,diff(datafile(:,5)*ticks2rads)/sampling_period,'r.-');
hold on;
right_encoder_data = diff(datafile(:,5)*ticks2rads)/sampling_period;
right_encoder_length = length(right_encoder_data);
K = mean(right_encoder_data((right_encoder_length-100):right_encoder_length));
tau63_index = max(find(right_encoder_data < K*.63));
tau28_index = max(find(right_encoder_data < K*.28));
tau63 = t(tau63_index);
tau28 = t(tau28_index);
tau = 3/2*(tau63-tau28);
tau = tau/2;
motorsis = tf(K/tau,[1 1/tau]);
[a,motort] = step(motorsis,t);
plot(motort,a,'k--');
title('Turtlebot 2 right motor identification');
xlabel('Absolute sensor timestamp (secs)');
ylabel('Wheel speed (rads/sec)');
legend('Right encoder','First order system Identification');
