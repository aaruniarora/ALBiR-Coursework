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

ramp_angles = 3:0.1:8; % Ramp angle range
initial_inter_leg_angle = 60;
initial_stance_angle = 15;
results = [];

% set_model_parameters(ramp_angle, initial_inter_leg_angle, initial_stance_angle)

%% Run Experiment
for ramp_angle = ramp_angles
    set_model_parameters(ramp_angle, initial_inter_leg_angle, initial_stance_angle)
    simOut = sim('PDW_Simulation', 'SimulationMode', 'normal', ...
            'StartTime', '0', 'StopTime', num2str(max_run_time), ...
            'Solver', 'ode23', 'MaxStep', num2str(maximum_step_size), ...
            'RelTol', num2str(relative_tolerance));

    % Extract inter-leg angle data (Rz.q)
    inter_leg_angles = simOut.logsout{1}.Values.Data; % Adjust index if needed
    time = simOut.logsout{1}.Values.Time;
    time_uniform = linspace(min(time), max(time), length(time));

    % Count steps
    steps = count_steps(inter_leg_angles, initial_inter_leg_angle, time_uniform);
    stepsCount = countsteps(initial_inter_leg_angle,inter_leg_angles,time_uniform);

    % Save results
    results = [results; ramp_angle, stepsCount];

    % Display progress
    fprintf('Ramp Angle: %.1f°, Steps: %d\n', ramp_angle, stepsCount);
end

% simOut = sim('PDW_Simulation', 'StopTime', '5');

% set_param('PDW_Simulation','SimulationCommand','start') % you can write stop instead if you ever want to stop the simulation
% disp(simOut.logsout); % Displays logged signals
% inter_leg_angles = simOut.logsout{1}.Values.Data; % Extract data (angles)
% time = simOut.logsout{1}.Values.Time; % Extract time
% angular_velocity = simOut.logsout{2}.Values.Data; % Extract angular velocity data

% plot(time, inter_leg_angles);
% plot(time, angular_velocity);
% xlabel('Time (s)');
% ylabel('Inter-Leg Angle (degrees)');
% title('Inter-Leg Angle vs. Time');

%% Get simulaiton output 

% % Plot the results
% figure;
% plot(results(:, 1), results(:, 2), '-o', 'LineWidth', 1.5);
% xlabel('Ramp Angle (degrees)');
% ylabel('Number of Steps');
% title('Steps vs. Ramp Angle');
% grid on;

% Plot the results
% figure;
% hold on;
% plot(results(:, 1), results(:, 2), 'b-', 'LineWidth', 1.5);
% for i = 1:size(results, 1)
%     if results(i, 2) >= 5
%         plot(results(i, 1), results(i, 2), 'ro', 'MarkerSize', 3, 'LineWidth', 1.5, 'MarkerFaceColor', 'r'); % Red marker for > 5 steps
%     else
%         plot(results(i, 1), results(i, 2), 'bo', 'MarkerSize', 3, 'LineWidth', 1.5); % Blue marker for ≤ 5 steps
%     end
% end
% xlabel('Ramp Angle (degrees)');
% ylabel('Number of Steps');
% title('Number of steps as a function of slope angle');
% grid on;
% hold off;

figure;
hold on;

% Main line plot (without legend)
plot(results(:, 1), results(:, 2), 'b-', 'LineWidth', 1.5);

% Loop through results for individual markers
for i = 1:size(results, 1)
    if results(i, 2) >= 5
        % Red marker for stable condition
        plot(results(i, 1), results(i, 2), 'ro', 'MarkerSize', 5, 'LineWidth', 1.5, 'MarkerFaceColor', 'r'); 
    else
        % Blue marker for normal condition
        plot(results(i, 1), results(i, 2), 'bo', 'MarkerSize', 5, 'LineWidth', 1.5); 
    end
end

% Add dummy plots for the legend
% plot(NaN, NaN, 'ro', 'MarkerSize', 5, 'LineWidth', 1.5, 'MarkerFaceColor', 'r'); % Red dot for Stable Condition
% plot(NaN, NaN, 'bo', 'MarkerSize', 5, 'LineWidth', 1.5); % Blue dot for Normal Condition

% Add labels and legend
xlabel('Ramp Angle (degrees)');
ylabel('Number of Steps');
title('Number of steps as a function of slope angle');
grid on;

% legend({'Stable Condition (#Steps > 5)', 'All Conditions'}, 'Location', 'northwest');

hold off;


% InterLegAng=out.simout; %first format
% InterLegAng=out.simout.signals.values; %second format

%% Step counter

% function steps = count_steps(inter_leg_angles, initial_inter_leg_angle, time)
%     threshold = 0.1 * initial_inter_leg_angle; % 10% of initial angle
%     crossings = (inter_leg_angles > threshold); % Logical array for crossings
%     % disp(crossings);
%     steps = sum(diff(crossings) == 1); % Count upward crossings
% 
%     % If no steps detected, check if it's because the walker fell immediately
%     if steps == 0 && length(time) < 5*0.5
%         steps = 0;  % Confirming zero steps for immediate falls
%     end
% end

function steps = count_steps(inter_leg_angles, initial_inter_leg_angle, time)
    % Define positive and negative thresholds based on the initial inter-leg angle
    threshold = 0.1 * initial_inter_leg_angle; % 10% of initial angle

    % Logical array for positive and negative crossings
    above_positive = inter_leg_angles > threshold; % Above +threshold
    below_negative = inter_leg_angles < -threshold; % Below -threshold

    my_sum = above_positive + below_negative;
    steps = sum(diff(my_sum) == 1);

    % If no steps detected, check if it's because the walker fell immediately
    if steps == 0 && length(time) < 5*0.5
        steps = 0;  % Confirming zero steps for immediate falls
    end
end

function stepsCount = countsteps(initial_inter_leg_angle,position,time)
    thresholdValue = 0.1 * initial_inter_leg_angle;
    crossedThreshold_down = false;
    crossedThreshold_up = false;
    stepsCount = 0;
    for i = 2:length(time)
        % Check if the inter-leg angle has crossed the lower threshold
        if position(i) < thresholdValue && position(i - 1) >= thresholdValue
            crossedThreshold_down = true;
        end

        % Check if the inter-leg angle has crossed the upper threshold
        if crossedThreshold_down && position(i) < -thresholdValue && position(i - 1) >= -thresholdValue
            stepsCount = stepsCount + 1;
            crossedThreshold_down = false;
        end
        % Check if the inter-leg angle has crossed the lower threshold
         if position(i) > -thresholdValue && position(i - 1) <= -thresholdValue
             crossedThreshold_up = true;
         end

         % Check if the inter-leg angle has crossed the upper threshold
         if crossedThreshold_up && position(i) > thresholdValue && position(i - 1) <= thresholdValue
             stepsCount = stepsCount + 1;
             crossedThreshold_up = false;
         end
    end
end