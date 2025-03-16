clc; clear; close all;

%% Load the dataset
% Define file names and frequencies
frequencies = 0.03:0.02:0.15;%0.05:0.05:0.50;%
% frequencies = [0.03:0.02:0.15, 0.05:0.05:0.40];
% frequencies = unique(sort(frequencies));

data_folder = 'Data/John_default/';  % John's 03:0.02:0.15
% data_folder = 'Data/John_tuned/';  % John's 0.05:0.05:0.60
% data_folder ='Data/Leo_p1_i005_d01/'; % Leo's 0.05:0.05:0.50
% data_folder ='Data/Leo_p1_i015_d01/'; % Leo's 0.05:0.05:0.50
% data_folder ='Data/Jenny/'; % Jenny's 0.03:0.02:0.13

% save_name = 'Images/john_default';
% save_name = 'Images/john_tuned';
% save_name = 'Images/leo_p1_i005_d01';
% save_name = 'Images/leo_p1_i015_d01';
% save_name = 'Images/jenny_p1_i015_d1';

% Initialize cell arrays to store data
num = length(frequencies);
run_time = cell(num,1);
error_signal = cell(num,1);
camera_angles = cell(num,1);
target_angles = cell(num,1);
t_uniform = cell(num,1);
t_new = cell(num,1);

error_amplitude = zeros(size(frequencies));
camera_amplitude = zeros(size(frequencies));
target_amplitude = zeros(size(frequencies));

gains = zeros(size(frequencies));
gain_dB = zeros(size(frequencies));
phases = zeros(size(frequencies));

% Create figure for subplots
figure (1); set(gcf, 'Position', get(0, 'Screensize'));
tiledlayout(round(num/2),2); % Creates a vertical grid of subplots

for f = 1:num
    % Generate file name 
    file_name = sprintf('%s%.2fhz.csv', data_folder, frequencies(f));
    
    % Read data
    T = 1/frequencies(f);
    data = readmatrix(file_name);
    % data = fillmissing(data, 'spline');
    
    % Extract columns
    run_time{f} = data(:,1) ./ 1000;  % Convert to seconds
    error_signal{f} = data(:,2); 
    camera_angles{f} = data(:,3); 

    % % Calculating smoothened data
    % error_signal{f} = movmean(data(:,2), 5);
    % camera_angles{f} = movmean(data(:,3), 5);
    
    % Mean normalisation to be centred around 0
    camera_angles{f} = camera_angles{f} - mean(camera_angles{f}); 
    error_signal{f} = error_signal{f} - mean(error_signal{f}); 

    % Find peaks in the camera_angles signal
    [pks, locs] = findpeaks(camera_angles{f}, MinPeakHeight=mean(camera_angles{f})); %fs=1/mean(diff(run_time{f})),

    % Ensure at least two peaks exist
    if length(locs) >= 2
        start_peak_idx = locs(2); % Select the second peak
        last_peak_idx = locs(end); % Last peak        
    end

    % Extract time shift from second peak
    t_shift = run_time{f}(start_peak_idx); 
    
    % Reset samples
    run_time{f} = run_time{f}(start_peak_idx:last_peak_idx); 
    error_signal{f} = error_signal{f}(start_peak_idx:last_peak_idx);
    camera_angles{f} = camera_angles{f}(start_peak_idx:last_peak_idx); 

    % Ensure Time Vector is Properly Aligned
    t_uniform{f} = linspace(min(run_time{f}), max(run_time{f}), length(run_time{f}));
    t_new{f} = min(run_time{f}):mean(diff(run_time{f})):max(run_time{f});

    % Generate Corrected Sine Wave with Peak Alignment
    A = 30; %(max(camera_angles{f}) - min(camera_angles{f})) / 2; % Peak amplitude
    target_angles{f} = A * sin(2 * pi * frequencies(f) * (t_uniform{f}.' - t_shift) - 3*pi/2);
    % target_angles{f} = camera_angles{f} + error_signal{f};
    
    %% Plot data for each frequency in a subplot
    nexttile;
    plot(t_uniform{f}, camera_angles{f}, 'LineWidth', 1.25, 'DisplayName', 'Given Angle'); hold on;
    plot(t_uniform{f}, error_signal{f}, 'LineWidth', 1, 'DisplayName', 'Error Angle');
    plot(t_uniform{f}, target_angles{f}, '--', 'LineWidth', 1.25, 'DisplayName', 'Calculated Angle');
    
    % Add labels and title for each subplot
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title(sprintf('Angles at %.2f Hz', frequencies(f)));
    legend show; grid on; grid minor;
end

%% Gain and Phase
figure (2); set(gcf, 'Position', get(0, 'Screensize'));
for i = 1:num
    %% Compute Amplitude and Gain
    % Compute weighted RMS amplitudes
    weighting_factor = 1 - abs(error_signal{i}) ./ max(abs(error_signal{i}));
    weighting_factor = weighting_factor(:); % Ensure column vector
    
    weighted_camera_angles = camera_angles{i};%(:) .* weighting_factor;
    % weighted_target_angles = target_angles{i}(:) .* weighting_factor; 

    camera_amplitude(i) = rms(weighted_camera_angles);
    % target_amplitude = rms(weighted_target_angles);
    % camera_amplitude = rms(camera_angles{i});
    target_amplitude(i) = rms(target_angles{i});
    error_amplitude(i) = rms(error_signal{i});
    
    % Compute Gain (normalized and filtered)
    gain = camera_amplitude(i) / target_amplitude(i);
    gain_dB(i) = 20 * log10(gain);
    gains(i) = gain;

    dt = mean(diff(run_time{i}));

    %% Phase Shift (1): Cross Correlation
    % Compute cross-correlation
    [corr_values, lag] = xcorr(weighted_camera_angles, target_angles{i}, 'coeff');

    % Find lag corresponding to maximum correlation
    [~, max_idx] = max(abs(corr_values));
    time_delay = lag(max_idx)' * mean(diff(run_time{i})); 

    % Compute phase shift in degrees
    phase_shift = 360 * (time_delay * frequencies(i));

    phases(i) = wrapTo180(-phase_shift); % Negative because phase lag

    %% Phase Shift (2): FFT
    % N = length(weighted_camera_angles);
    % Y1 = fft(weighted_camera_angles);
    % Y2 = fft(target_angles{i});
    % f_axis = (0:N-1)*(1/(N*dt));  % frequency vector
    % 
    % % Find the index closest to the desired frequency f:
    % [~, idx] = min(abs(f_axis - f));
    % 
    % phase1 = angle(weighted_camera_angles(idx));
    % phase2 = angle(target_angles{i}(idx));
    % 
    % phase_diff_rad = phase1 - phase2;
    % phases(i) = wrapTo180(rad2deg(phase_diff_rad));
    % % fprintf('FFT Phase difference = %.2f degrees\n', phases(i));

    %% Phase (3): Hilbert Transform
    % analytic1 = hilbert(weighted_camera_angles);
    % analytic2 = hilbert(target_angles{i});
    % 
    % phase1 = unwrap(angle(analytic1));
    % phase2 = unwrap(angle(analytic2));
    % 
    % % Compute instantaneous phase difference (in radians), then convert to degrees.
    % inst_phase_diff = rad2deg(phase1 - phase2);
    % 
    % % For a single value, you might average over a steady region:
    % phases(i) = wrapTo180(mean(inst_phase_diff(round(end/2):end)));
    % % fprintf('Hilbert Transform Phase difference = %.2f degrees\n', phase_diff_deg);

    %% Phase (4): Zero-crossing
    % % Assume you have a function to compute zero-crossings:
    % zc_idx1 = find(diff(sign(weighted_camera_angles)) ~= 0);
    % zc_idx2 = find(diff(sign(target_angles{i})) ~= 0);
    % 
    % % For simplicity, assume the first zero-crossing in each signal corresponds:
    % t_zc1 = (zc_idx1(1) + interpZero(weighted_camera_angles, zc_idx1(1), dt));
    % t_zc2 = (zc_idx2(1) + interpZero(target_angles{i}, zc_idx2(1), dt));
    % 
    % time_delay = t_zc1 - t_zc2;
    % phase_deg = wrapTo180(360 * f * time_delay);
    % % fprintf('Zero-crossing Phase difference = %.2f degrees\n', phase_deg);

    %% Phase (5): Sine wave fitting
    % % Define sine wave fitting function
    % sine_fit = @(b, t) b(1) * sin(2 * pi * b(2) * t + b(3)); % b = [Amplitude, Frequency, Phase]
    % 
    % % Initial guess: [Amplitude, Frequency, Phase]
    % freq_guess = frequencies(i); % Use the known frequency
    % amp_guess_cam = (max(weighted_camera_angles) - min(weighted_camera_angles)) / 2;
    % amp_guess_target = (max(target_angles{i}) - min(target_angles{i})) / 2;
    % 
    % % Fit to weighted_camera_angles
    % b0_cam = [amp_guess_cam, freq_guess, 0]; % Initial guess
    % cam_fit = lsqcurvefit(sine_fit, b0_cam, run_time{i}, weighted_camera_angles);
    % 
    % % Fit to target_angles
    % b0_target = [amp_guess_target, freq_guess, 0]; % Initial guess
    % target_fit = lsqcurvefit(sine_fit, b0_target, run_time{i}, target_angles{i});
    % 
    % % Extract phase values
    % phi_camera = cam_fit(3);
    % phi_target = target_fit(3);
    % 
    % % Compute phase shift in degrees
    % phase_shift = rad2deg(phi_target - phi_camera);
    % 
    % % Generate fitted sine wave for plotting
    % time_fine = linspace(min(run_time{i}), max(run_time{i}), 1000); % High-resolution time vector
    % cam_fitted_wave = sine_fit(cam_fit, time_fine);
    % target_fitted_wave = sine_fit(target_fit, time_fine);
    % 
    % % Plot fitted sine waves for each frequency
    % subplot(round(num/2),2,i); % Creates a vertical grid of subplots
    % 
    % plot(run_time{i}, weighted_camera_angles, 'bo', 'MarkerSize', 2.5, 'DisplayName', 'Camera Data'); hold on;
    % plot(run_time{i}, target_angles{i}, 'ro', 'MarkerSize', 2.5, 'DisplayName', 'Target Data');
    % plot(time_fine, cam_fitted_wave, 'b-', 'LineWidth', 1, 'DisplayName', 'Camera Fit');
    % plot(time_fine, target_fitted_wave, 'r-', 'LineWidth', 1, 'DisplayName', 'Target Fit');
    % 
    % xlabel('Time (s)');
    % ylabel('Angle');
    % title(sprintf('Sine Fit at %.2f Hz', frequencies(i)));
    % legend; grid on;
    % phases(i) = wrapTo180(-phase_shift); % Negative because phase lag

end

%% RMS plot
disp('Servo'); disp(camera_amplitude);
disp('Error'); disp(error_amplitude);

figure(3);
set(gcf, 'Position', get(0, 'Screensize')); % Make figure full-screen

% Set default font size
set(gca, 'FontSize', 11); hold on;
plot(frequencies, camera_amplitude, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Servo Tracking'); 
plot(frequencies, error_amplitude, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Visual Error');
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Amplitude (RMS)', 'FontSize', 11);
title('Servo Tracking and Visual Error Amplitude', 'FontSize', 11);
grid on; legend show; hold off;

% Save high-resolution figures
% exportgraphics(gcf, [save_name '_rms.png'], 'Resolution', 600);
% exportgraphics(gcf, [save_name '_rms.pdf'], 'ContentType', 'vector');
% print(gcf, [save_name '_rms.eps'], '-depsc', '-r600'); 

%% Bode Plot
figure(4);
set(gcf, 'Position', get(0, 'Screensize')); % Make figure full-screen

% Gain Plot
subplot(2,1,1);
semilogx(frequencies, gains, 'o-', 'LineWidth', 1.5, 'Color', '#001F70');
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Gain (Normalised)', 'FontSize', 11);
title('Bode Plot: Gain', 'FontSize', 11);
grid on;
set(gca, 'XScale', 'log', 'FontSize', 11); 

% Phase Plot
subplot(2,1,2);
semilogx(frequencies, phases, 'o-', 'LineWidth', 1.5, 'Color', '#001F70');
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('Phase Lag (degrees)', 'FontSize', 11);
title('Bode Plot: Phase', 'FontSize', 11);
grid on;
set(gca, 'XScale', 'log', 'FontSize', 11); 

% Save high-resolution figures
% exportgraphics(gcf, [save_name '_nodB.png'], 'Resolution', 600);
% exportgraphics(gcf, [save_name '_nodB.pdf'], 'ContentType', 'vector');
% print(gcf, [save_name '_nodB.eps'], '-depsc', '-r600'); 

%% 
% Helper function (simple linear interpolation for zero crossing)
function offset = interpZero(sig, idx, dt)
    % Linear interpolation offset within the sample interval to reach zero.
    x1 = sig(idx);
    x2 = sig(idx+1);
    offset = dt * (-x1/(x2-x1));
end