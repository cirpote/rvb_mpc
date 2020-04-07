clc, close all, clear all;

log_data = importdata('../log_output_folder/log_output_0.txt');
log_data_val = importdata('../log_output_folder/log_output_1.txt');

delta_t = 0.01;

pitch_axis = [log_data(:,1) log_data(:,5)];
yaw_axis = [log_data(:,2) log_data(:,5)];
pitch_axis_cmd = [log_data(:,3) log_data(:,5)];
yaw_axis_cmd = [log_data(:,4) log_data(:,5)];

time = linspace(0, log_data(end,5), log_data(end,5) / delta_t + 2);
pitch_axis_int = interp1(pitch_axis(:,2), pitch_axis(:,1), time)'; pitch_axis_int(isnan(pitch_axis_int)) = 0;
yaw_axis_int = interp1(yaw_axis(:,2), yaw_axis(:,1), time)'; yaw_axis_int(isnan(yaw_axis_int)) = 0;
pitch_axis_cmd_int = interp1(pitch_axis_cmd(:,2), pitch_axis_cmd(:,1), time)'; pitch_axis_cmd_int(isnan(pitch_axis_cmd_int)) = 0;
yaw_axis_cmd_int = interp1(yaw_axis_cmd(:,2), yaw_axis_cmd(:,1), time)'; yaw_axis_cmd_int(isnan(yaw_axis_cmd_int)) = 0;


pitch_axis_val = [log_data_val(:,1) log_data_val(:,5)];
yaw_axis_val = [log_data_val(:,2) log_data_val(:,5)];
pitch_axis_cmd_val = [log_data_val(:,3) log_data_val(:,5)];
yaw_axis_cmd_val = [log_data_val(:,4) log_data_val(:,5)];

time_yaw = linspace(0, log_data_val(end,5), log_data_val(end,5) / delta_t + 2); 
pitch_axis_int_val = interp1(pitch_axis_val(:,2), pitch_axis_val(:,1), time_yaw)'; pitch_axis_int_val(isnan(pitch_axis_int_val)) = 0;
yaw_axis_int_val = interp1(yaw_axis_val(:,2), yaw_axis_val(:,1), time_yaw)'; yaw_axis_int_val(isnan(yaw_axis_int_val)) = 0;
pitch_axis_cmd_int_val = interp1(pitch_axis_cmd_val(:,2), pitch_axis_cmd_val(:,1), time_yaw)'; pitch_axis_cmd_int_val(isnan(pitch_axis_cmd_int_val)) = 0;
yaw_axis_cmd_int_val = interp1(yaw_axis_cmd_val(:,2), yaw_axis_cmd_val(:,1), time_yaw)'; yaw_axis_cmd_int_val(isnan(yaw_axis_cmd_int_val)) = 0;


% pitch_tau = 0.3;
% pitch_gain = 1;
% pitch_sim = 0;
% for i=1:length(time) - 1
% 
%     delta_pitch = ( 1 / pitch_tau) * (pitch_gain * pitch_axis_cmd_int(i) - pitch_sim(i) );
%     pitch_sim = [pitch_sim; pitch_sim(i) + delta_pitch * delta_t];
%     
% end
% 
% 
% plot(time, pitch_axis_int, 'r');
% hold on;
% plot(time, pitch_axis_cmd_int, 'g');
% plot(time, pitch_sim);

yaw_tau = 0.45;
yaw_gain = 1;
yaw_sim = 0;
for i=1:length(time_yaw) - 1

    delta_yaw = ( 1 / yaw_tau) * (yaw_gain * yaw_axis_cmd_int_val(i) - yaw_sim(i) );
    yaw_sim = [yaw_sim; yaw_sim(i) + delta_yaw * delta_t];
    
end


plot(time_yaw, yaw_axis_int_val, 'r');
hold on;
plot(time_yaw, yaw_axis_cmd_int_val, 'g');
plot(time_yaw, yaw_sim);




