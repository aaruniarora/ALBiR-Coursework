%----------------------------------------------------------
% Example data setup
%----------------------------------------------------------
% Suppose you have 5 different input distances, each tested 5 times.
inputDistances = [10; 15; 17.5; 20; 22.5; 25; 27.5; 30]; % Column vector (5x1)

% "actualDistances" is a 5x5 matrix where each row corresponds
% to an input distance, and each column is one of the 5 trials.
actualDistances = [ 9.7, 9.9, 9.9, 10, 9.8;   % Trials for distance=10
                    16.1, 15.2, 15.0, 14.5, 15.5;   % Trials for distance=15
                    18.5, 18, 18.6, 18.3, 18.6; % Trials for distance=17.5
                    19.9, 20, 19.5, 19.2, 19; % Trials for distance=20
                    22.7, 24.2, 23.5, 22.5, 22.5; % Trials for distance=22.5
                    23.5, 21.2, 25.5, 27, 27.3; % Trials for distance=25
                    29.5, 29.5, 27.5, 30.9, 30.5; % Trials for distance=27.5
                    30, 29.4, 29.7, 31, 28]; % Trials for distance=30

% "predictedDistances" is also 5x5 in the same layout.
predictedDistances = [ 9.4, 8.86, 9.7, 9.7, 9.7;
                       14.95, 14.23, 14.68, 14, 14.6;
                       17.06, 16.5, 17.4, 17.2, 17.4;
                       19.2, 19.6, 19.6, 19.5, 19.4;
                       21.8, 22.4, 22.3, 23, 22.4;
                       21.98, 21.1, 23.2, 24.1, 24;
                       26.15, 26.9, 24.3, 26.2, 26.2;
                       26.4, 26.5, 27, 27.1, 24];
                   
%----------------------------------------------------------
% Compute means and standard deviations
%----------------------------------------------------------
meanActual     = mean(actualDistances, 2);   % average across the 5 trials
stdActual      = std(actualDistances, 0, 2); % standard deviation
meanPredicted  = mean(predictedDistances, 2);
stdPredicted   = std(predictedDistances, 0, 2);

%----------------------------------------------------------
% Plotting with error bars
%----------------------------------------------------------
figure;
hold on;

% % Plot input distance
plot(inputDistances, 'LineWidth', 1.5, 'DisplayName', 'Input')
% errorbar(inputDistances, inputDistances, 0, 'LineWidth', 1.5, 'DisplayName', 'Input');

% Plot the actual distances (with error bars)
% errorbar(inputDistances, meanActual, stdActual, 'LineWidth', 1.5, 'DisplayName', 'Actual');
plot(meanActual, 'LineWidth', 1.5, 'DisplayName', 'Actual')

% Plot the predicted distances (with error bars)
% errorbar(inputDistances, meanPredicted, stdPredicted, 'LineWidth', 1.5, 'DisplayName', 'Predicted');
plot(meanPredicted, 'LineWidth', 1.5, 'DisplayName', 'Predicted')

hold off;
grid on;

% xlabel('Input Distance');
% ylabel('Measured/Predicted Distance');
title('Comparison of Robot Measured vs. Predicted Distances');
% legend('Actual','Predicted','Location','Best');
legend show; grid on;

