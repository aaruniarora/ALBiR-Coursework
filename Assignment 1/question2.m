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
initial_inter_leg_angles = 34:4:54;
initial_stance_angles = 3:3:20;

% Create a matrix to store results for surface plot
results_matrix = zeros(length(initial_inter_leg_angles), length(initial_stance_angles));

%% Run Experiment
for i = 1:length(initial_inter_leg_angles)
    for j = 1:length(initial_stance_angles)
        initial_inter_leg_angle = initial_inter_leg_angles(i);
        initial_stance_angle = initial_stance_angles(j);

        set_model_parameters(ramp_angle, initial_inter_leg_angle, initial_stance_angle)
        simOut = sim('PDW_Simulation', 'SimulationMode', 'normal', ...
                'StartTime', '0', 'StopTime', num2str(max_run_time), ...
                'Solver', 'ode23', 'MaxStep', num2str(maximum_step_size), ...
                'RelTol', num2str(relative_tolerance));

        % Extract inter-leg angle data (Rz.q)
        inter_leg_angles = simOut.logsout{1}.Values.Data; % Adjust index if needed
        time = simOut.logsout{1}.Values.Time;

        % Count steps
        steps = count_steps(inter_leg_angles, initial_inter_leg_angle, time);

        % Store steps in results matrix
        results_matrix(i, j) = steps;

        fprintf('Initial Leg Angle: %.1f°, Initial Stance Angle: %.1f°, Steps: %d\n', initial_inter_leg_angle, initial_stance_angle, steps);

    end
end

%% Create a surface plot
[X, Y] = meshgrid(initial_stance_angles, initial_inter_leg_angles);

figure(1);
surf(X, Y, results_matrix);
% shading interp
xlabel('Initial Stance Angle (degrees)');
ylabel('Initial Inter-Leg Angle (degrees)');
zlabel('Number of Steps');
title('Surface Plot for number of steps as a function of the initial stance angle and inter-leg angle');
grid on;

exportgraphics(gca, 'lecture2_matsave.png', 'Resolution', 600); % Save with 300 DPI

%% Step counter function
function steps = count_steps(inter_leg_angles, initial_inter_leg_angle, time)
    % Define thresholds based on the initial inter-leg angle
    threshold = 0.1 * initial_inter_leg_angle; % 10% of initial angle

    % Logical array for crossings
    above_positive = inter_leg_angles > threshold; % Above +threshold
    below_negative = inter_leg_angles < -threshold; % Below -threshold

    % Count step transitions
    my_sum = above_positive + below_negative;
    steps = sum(diff(my_sum) == 1);

    % Handle cases where the walker falls immediately
    if steps == 0 && length(time) < 5 * 0.5
        steps = 0;
    end
end