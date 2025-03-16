clc; clear; close all;

%% Load the dataset
today = datetime('today');
saveplot = 0; % saves as .eps, .png, .pdf if 1 for RMS, 2 for bode and 3 for both.
fts = 13; % font size for graphs
method = 'tuned'; % default or tuned dataset

%%
if strcmp(method, 'default')
    fail_freq = 0.13; % Hz
    plottitle = '(P=0.2, I=0.0, D=0.005)'; 
    frequencies = 0.03:0.02:0.15;%0.05:0.05:0.50;%
    data_folder = 'Data/John_default/';  % John's 03:0.02:0.15 (P=0.2, I=0.0, D=0.005)
    save_name = sprintf('Images/john_default_%s', today);
end

if strcmp(method, 'tuned')
    fail_freq = 0.9; % Hz
    plottitle = '(P=1.0, I=0.05, D=0.05)'; 
    frequencies = [0.03:0.02:0.15, 0.05:0.05:0.70, 0.8, 0.9, 1.2];
    frequencies = unique(sort(frequencies));
    data_folder = 'Data/John_tuned_new/';  % John's 03:0.02:0.15  (P=1.0, I=0.05, D=0.05)
    save_name = sprintf('Images/john_tuned_%s', today);
end

%% Initialisation 

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
tiledlayout(round(num/4),4); % Creates a vertical grid of subplots

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
    % target_angles{f} = A * sin(2 * pi * frequencies(f) * (t_uniform{f}.' - t_shift) - 3*pi/2);
    target_angles{f} = camera_angles{f} + error_signal{f};
    
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

    camera_amplitude(i) = rms(weighted_camera_angles);
    target_amplitude(i) = rms(target_angles{i});
    error_amplitude(i) = rms(error_signal{i});
    
    % Compute Gain (normalized and filtered)
    gain = camera_amplitude(i) / target_amplitude(i);
    gain_dB(i) = 20 * log10(gain);
    gains(i) = gain;

    %% Phase Shift (1): Cross Correlation
    % % Compute cross-correlation
    % [corr_values, lag] = xcorr(weighted_camera_angles, target_angles{i}, 'coeff');
    % 
    % % Find lag corresponding to maximum correlation
    % [~, max_idx] = max(abs(corr_values));
    % time_delay = lag(max_idx)' * mean(diff(run_time{i})); 
    % 
    % % Compute phase shift in degrees
    % phase_shift = 360 * (time_delay * frequencies(i));

    % phases(i) = wrapTo180(-phase_shift); % Negative because phase lag

    %% Phase Shift (2): Sine wave fitting
    % Define sine wave fitting function
    sine_fit = @(b, t) b(1) * sin(2 * pi * b(2) * t + b(3)); % b = [Amplitude, Frequency, Phase]

    % camera_angles{f} = sgolayfilt(camera_angles{f}, 3, fts);
    % target_angles{f} = sgolayfilt(target_angles{f}, 3, fts);

    % Initial guess: [Amplitude, Frequency, Phase]
    freq_guess = frequencies(i); % Use the known frequency
    amp_guess_cam = (max(weighted_camera_angles) - min(weighted_camera_angles)) / 2;
    amp_guess_target = (max(target_angles{i}) - min(target_angles{i})) / 2;

    % Parameter Estimation
    lb_cam = [0.5 * amp_guess_cam, 0.8 * freq_guess, -pi, -10];
    ub_cam = [2 * amp_guess_cam, 1.2 * freq_guess, pi, 10];
    lb_target = [0.5 * amp_guess_target, 0.8 * freq_guess, -pi, -10];
    ub_target = [2 * amp_guess_target, 1.2 * freq_guess, pi, 10];

    % Fit to weighted_camera_angles
    b0_cam = [amp_guess_cam, freq_guess, 0]; % Initial guess
    cam_fit = lsqcurvefit(sine_fit, b0_cam, run_time{i}, weighted_camera_angles, lb_cam, ub_cam);

    % Fit to target_angles
    b0_target = [amp_guess_target, freq_guess, 0]; % Initial guess
    target_fit = lsqcurvefit(sine_fit, b0_target, run_time{i}, target_angles{i}, lb_target, ub_target);

    % Extract phase values
    phi_camera = cam_fit(3);
    phi_target = target_fit(3);

    % Compute phase shift in degrees
    phase_shift = rad2deg(phi_target - phi_camera);

    % Generate fitted sine wave for plotting
    time_fine = linspace(min(run_time{i}), max(run_time{i}), 1000); % High-resolution time vector
    cam_fitted_wave = sine_fit(cam_fit, time_fine);
    target_fitted_wave = sine_fit(target_fit, time_fine);

    % Plot fitted sine waves for each frequency
    subplot(round(num/4),4,i); % Creates a vertical grid of subplots

    plot(run_time{i}, weighted_camera_angles, 'bo', 'MarkerSize', 2.5, 'DisplayName', 'Camera Data'); hold on;
    plot(run_time{i}, target_angles{i}, 'ro', 'MarkerSize', 2.5, 'DisplayName', 'Target Data');
    plot(time_fine, cam_fitted_wave, 'b-', 'LineWidth', 1, 'DisplayName', 'Camera Fit');
    plot(time_fine, target_fitted_wave, 'r-', 'LineWidth', 1, 'DisplayName', 'Target Fit');

    xlabel('Time (s)');
    ylabel('Angle');
    title(sprintf('Sine Fit at %.2f Hz', frequencies(i)));
    legend; grid on;
    phases(i) = -phase_shift; % Negative because phase lag

end

%% RMS plot
disp('Servo'); disp(camera_amplitude);
disp('Error'); disp(error_amplitude);
disp('Mean diff'); disp(abs((camera_amplitude - error_amplitude)/error_amplitude))

figure(3); subplot(1,3,1); sgtitle(sprintf('%s', plottitle))
set(gcf, 'Position', get(0, 'Screensize')); % Make figure full-screen

% Set default font size
set(gca, 'FontSize', fts); hold on;
plot(frequencies, camera_amplitude, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Servo Tracking'); 
plot(frequencies, error_amplitude, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Visual Error');
xline(fail_freq, '--r', 'LineWidth',1.2, DisplayName='Fail');
xlabel('Frequency (Hz)', 'FontSize', fts, 'Interpreter', 'latex');
ylabel('RMS Amplitude (degrees)', 'FontSize', fts, 'Interpreter', 'latex');
title(sprintf('Servo Tracking vs Visual Error %s', plottitle), 'FontSize', fts+2, 'Interpreter', 'latex');
grid on; legend('show', 'Location', 'best'); hold off;

% Save high-resolution figures
if saveplot == 1 || saveplot == 3
    exportgraphics(gcf, [save_name '_rms.png'], 'Resolution', 600);
    exportgraphics(gcf, [save_name '_rms.pdf'], 'ContentType', 'vector');
    % exportgraphics(gcf, [save_name '_rms.svg'], 'ContentType', 'vector');
    print(gcf, [save_name '_rms.eps'], '-depsc', '-r600'); 
end

%% Bode Plot
figure(3);
set(gcf, 'Position', get(0, 'Screensize')); % Make figure full-screen

% Gain Plot
subplot(1,3,2);
semilogx(frequencies, gain_dB, 'o-', 'LineWidth', 1.5, 'Color', '#001F70'); hold on;
xline(fail_freq, '--r', 'LineWidth',1.2, DisplayName='Fail');
xlabel('Frequency (Hz)', 'FontSize', fts, 'Interpreter', 'latex');
ylabel('Gain (dB)', 'FontSize', fts, 'Interpreter', 'latex');
title(sprintf('Bode Plot: Gain  %s', plottitle), 'FontSize', fts+2, 'Interpreter', 'latex');
grid on; %legend('show', 'Location', 'best');
set(gca, 'XScale', 'log'); hold off;

% Phase Plot
subplot(1,3,3);
semilogx(frequencies, phases, 'o-', 'LineWidth', 1.5, 'Color', '#001F70'); hold on;
xline(fail_freq, '--r', 'LineWidth',1.2, DisplayName='Fail');
xlabel('Frequency (Hz)', 'FontSize', fts, 'Interpreter', 'latex');
ylabel('Phase Lag (degrees)', 'FontSize', fts, 'Interpreter', 'latex');
title(sprintf('Bode Plot: Phase  %s', plottitle), 'FontSize', fts+2, 'Interpreter', 'latex');
grid on; %legend('show', 'Location', 'best');
set(gca, 'XScale', 'log'); hold off;

%% Calculate Stability Margins from Bode Data
% --- Gain Crossover Frequency (GCF) ---
% Find index where gain_dB changes sign (crossing 0 dB)
idx_gcf = find(gain_dB(1:end-1) .* gain_dB(2:end) < 0, 1, 'first');
if ~isempty(idx_gcf)
    % Linear interpolation between two points:
    f1 = frequencies(idx_gcf);
    f2 = frequencies(idx_gcf+1);
    g1 = gain_dB(idx_gcf);
    g2 = gain_dB(idx_gcf+1);
    % Fraction between points for 0 dB crossing:
    frac = (0 - g1) / (g2 - g1);
    f_gcf = f1 + frac*(f2 - f1);
    
    % Interpolate phase at this frequency:
    p1 = phases(idx_gcf);
    p2 = phases(idx_gcf+1);
    phase_at_gcf = p1 + frac*(p2 - p1);
    
    % Compute phase margin: PM = (phase at gain crossover) + 180.
    PM = phase_at_gcf + 180;
else
    f_gcf = NaN;
    phase_at_gcf = NaN;
    PM = NaN;
end

% --- Phase Crossover Frequency (PCF) ---
% Find index where phase crosses -180°.
% We search for a sign change in (phase + 180).
idx_pcf = find((phases(1:end-1) + 180) .* (phases(2:end) + 180) < 0, 1, 'first');
if ~isempty(idx_pcf)
    f1 = frequencies(idx_pcf);
    f2 = frequencies(idx_pcf+1);
    p1 = phases(idx_pcf);
    p2 = phases(idx_pcf+1);
    frac = (-180 - p1) / (p2 - p1);
    f_pcf = f1 + frac*(f2 - f1);
    
    % Interpolate gain (in dB) at phase crossover frequency:
    g1 = gain_dB(idx_pcf);
    g2 = gain_dB(idx_pcf+1);
    gain_at_pcf = g1 + frac*(g2 - g1);
    
    % Compute gain margin: GM = - (gain at phase crossover)
    GM = -gain_at_pcf;
else
    f_pcf = NaN;
    gain_at_pcf= NaN;
    GM = NaN;
end

% %% Annotate the Bode Plot (Figure 4) with Margins and Vertical Lines
% figure(4);
% 
% % --- Phase Margin Annotation ---
% subplot(1,2,2);  % Phase plot
% hold on;
% if ~isnan(f_gcf)
%     % Mark the point
%     plot(f_gcf, phase_at_gcf, 'ks', 'MarkerSize',10, 'MarkerFaceColor','k');
%     % Draw a vertical line at f_gcf
%     xline(f_gcf, '--k', 'LineWidth',1.2);
%     % Text label
%     text(f_gcf, phase_at_gcf, sprintf('  PM = %.2f° @ %.2f Hz', PM, f_gcf), ...
%         'VerticalAlignment','bottom', 'HorizontalAlignment','left','FontSize', 11, 'Interpreter', 'latex');
% end
% if ~isnan(f_pcf)
%     % Draw a vertical line at f_pcf
%     xline(f_pcf, '--k', 'LineWidth',1.2);
% end
% 
% % --- Gain Margin Annotation ---
% subplot(1,2,1);  % Gain plot
% hold on;
% if ~isnan(f_pcf)
%     % Mark the point
%     plot(f_pcf, gain_at_pcf, 'ks', 'MarkerSize',10, 'MarkerFaceColor','k');
%     % Draw a vertical line at f_pcf
%     xline(f_pcf, '--k', 'LineWidth',1.2);
%     % Text label
%     text(f_pcf, gain_at_pcf, sprintf('  GM = %.2f dB @ %.2f Hz', GM, f_pcf), ...
%         'VerticalAlignment','bottom', 'HorizontalAlignment','left','FontSize', 11, 'Interpreter', 'latex');
% end
% if ~isnan(f_gcf)
%     % Draw a vertical line at f_gcf
%     xline(f_gcf, '--k', 'LineWidth',1.2);
% end
% 
% % Save high-resolution figures
% if saveplot == 2 || saveplot == 3
%     exportgraphics(gcf, [save_name '.png'], 'Resolution', 600);
%     exportgraphics(gcf, [save_name '.pdf'], 'ContentType', 'vector');
%     % exportgraphics(gcf, [save_name '.svg'], 'ContentType', 'vector');
%     print(gcf, [save_name '.eps'], '-depsc', '-r600'); 
% end