clear all
close all
clc

%% Initialise fixed parameters

max_run_time = 5;
maximum_step_size = 0.001;
relative_tolerance = 1e-3;
PDW_Simulation_DataFile4
set_param(bdroot,'Solver','ode23')
mass = 0.029;


%% Initialise variable parameters

ramp_angle = 4;
ramp_angle_rad = deg2rad(ramp_angle); % Convert ramp angle to radians

initial_inter_leg_angle = 42;
initial_stance_angle = 3;
g = 9.81; % Gravitational acceleration

set_model_parameters(ramp_angle, initial_inter_leg_angle, initial_stance_angle)

%% Run Experiment

% set_param('PDW_Simulation','SimulationCommand','start') % you can write stop instead if you ever want to stop the simulation
simout = sim('PDW_Simulation', 'StopTime', '3');

%% Extract CoM data

time = simout.logsout{1}.Values.Time; % Extract time
x_CoM = simout.CoM_out(:,1); % CoM positions x
y_CoM = simout.CoM_out(:,2); % CoM positions x
z_CoM = simout.CoM_out(:,3); % CoM positions x
disp('Extracted');

%% Filtering

% % Design a low-pass FIR filter
% d = designfilt('lowpassfir', 'PassbandFrequency', 0.1, ...
%                'StopbandFrequency', 0.15, 'PassbandRipple', 1, ...
%                'StopbandAttenuation', 60, 'DesignMethod', 'kaiserwin');
% 
% % Filter the CoM data
% x_CoM_filt = filtfilt(d, x_CoM);
% y_CoM_filt = filtfilt(d, y_CoM);
% z_CoM_filt = filtfilt(d, z_CoM);

%% Butterworth

% Resample the trajectory to a uniform time vector (e.g., 0.001s step)
time_uniform = linspace(min(time), max(time), length(time));
time_uniform = time_uniform';
% x_resampled = interp1(time, x_CoM, time_uniform);
% y_resampled = interp1(time, y_CoM, time_uniform);
% z_resampled = interp1(time, z_CoM, time_uniform);

% Optionally apply a low-pass filter to smooth data
fs = 1 / (time_uniform(2) - time_uniform(1)); % Sampling frequency
fc = 12.5; % Cutoff frequency (Hz)
[b, a] = butter(2, fc / (fs / 2), 'low'); % 2nd-order Butterworth filter
x_CoM_filt = filtfilt(b, a, x_CoM);
y_CoM_filt = filtfilt(b, a, y_CoM);
z_CoM_filt = filtfilt(b, a, z_CoM);

disp('Filtering done');

%% Check plot for trajectories 
% figure (1)
% hold on
% plot(time, x_CoM_filt);
% plot(time, y_CoM_filt);
% plot(time, z_CoM_filt);
% xlabel('Time (s)');
% ylabel('Inter-Leg Angle (degrees)');
% title('Inter-Leg Angle vs. Time'); legend('x', 'y', 'z');
% grid on
% hold off

%% Get simulaiton output 

% InterLegAng=out.simout; %first format
% InterLegAng=out.simout.signals.values; %second format

%% PE

% Compute the height (adjusted for ramp angle)
h = y_CoM_filt - tan(ramp_angle_rad) * x_CoM_filt;

% Compute potential energy
PE = mass * g * z_CoM_filt;

%% PE Detrend

% % Fit a linear trend to the PE
% p = polyfit(time_uniform, PE, 1);
% PE_trend = polyval(p, time_uniform);
% 
% % De-trend PE
% PE_detrended = PE - PE_trend;
num_indices = length(PE);
disp(num_indices);
index = 24500;
% Detrend only the first 'index' elements
PE_detrended_part = detrend(PE(1:index), 'linear');

% Concatenate with the remaining unchanged part
PE_detrended = [PE_detrended_part; PE(index+1:end)];

disp('PE done');

%% KE

% Compute velocities using numerical differentiation
vx = gradient(x_CoM_filt, mean(diff(time_uniform)));
vy = gradient(y_CoM_filt, mean(diff(time_uniform)));
vz = gradient(z_CoM_filt, mean(diff(time_uniform)));
v = sqrt(vx.^2 + vy.^2 + vz.^2); % Velocity magnitude

% v_smooth = filtfilt(b, a, v); % Smooth velocity

% Append last value to match dimensions
% v = [v; v(end)];

% Compute kinetic energy
KE = 0.5 * mass * v.^2;
disp('KE done');

%% HeelStrike

% Retrieve heel strike event times
% HeelStrikeEvent = logsout.get('HeelStrikeEvent').Values.Data;
% HeelStrikeTimes = time(HeelStrikeEvent == 1);
% time = simOut.StanceLegAngle.Time; % Time vector
% stance_leg_angle = simOut.StanceLegAngle.Data; % Stance leg angle data
% 
% % Compute the derivative of the stance leg angle
% angle_derivative = gradient(stance_leg_angle, time);
% 
% % Detect zero-crossings of the derivative (local minima or maxima)
% heel_strike_indices = find(diff(sign(angle_derivative)) < 0);
% 
% % Get the times of heel strikes
% heel_strike_times = time(heel_strike_indices);

% Use the islocalmin function to find minima in the vertical CoM
heel_strike_indices = islocalmin(z_CoM_filt);

% Get the times of heel strikes
heel_strike_times = time_uniform(heel_strike_indices);
disp('heel done');

figure(1);
plot(time_uniform, z_CoM_filt)
xlabel('Time (s)');
ylabel('CoM z-axis values');
title('CoM z-axis trajectory over time to determine local minima');
grid on;

%% Figure

figure (2);
plot(time_uniform, KE, 'r', 'LineWidth', 1.5); hold on;
plot(time_uniform, PE_detrended, 'b', 'LineWidth', 1.5);

% Annotate heel strikes
for t = heel_strike_times
    xline(t, '--k')%, 'Heel Strike', 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
end

legend('Kinetic Energy (KE)', 'De-Trended Potential Energy (PE)', 'Heel Strike');
xlabel('Time (s)');
ylabel('Energy (J)');
title('Phase Relationship Between KE and De-Trended PE');
grid on;