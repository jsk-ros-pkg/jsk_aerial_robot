clc; clear; close all;
% Define the file path
file_path = '~/.ros/servo=0_u=23.2v_motor_test_1718360916.txt';

% Read the data from the file
data = readtable(file_path, 'Delimiter', ' ', 'ReadVariableNames', false);

% Assign column names
data.Properties.VariableNames = {'PWM', 'fx', 'fy', 'fz', 'f_norm', 'mx', 'my', 'mz', 'currency', 'RPM', 'temperature', 'voltage', 'State'};

% Extract data into individual variables
PWM = data.PWM;
fx = data.fx;
fy = data.fy;
fz = data.fz;
f_norm = data.f_norm;
mx = data.mx;
my = data.my;
mz = data.mz;
currency = data.currency;
RPM = data.RPM;
temperature = data.temperature;
voltage = data.voltage;
State = data.State;

% Display the first few rows of the table for verification
disp(head(data));

% Define frequency (Hz)
freq = 50;

% Generate time data
num_samples = height(data); % Get the number of rows in the data
time = linspace(0, num_samples / freq, num_samples);

% Construct SYSID data
time_sysid = time(199.5*50:202*50);
fz_sysid = fz(199.5*50:202*50);
cmd_sysid = PWM(199.5*50:202*50);
cmd_sysid(cmd_sysid <= 1650) = 0;
cmd_sysid(cmd_sysid > 1650) = 14;

plot(time_sysid, fz_sysid)
hold on
plot(time_sysid, cmd_sysid)
