clc; clear; close all;
% --------------------------------------------------------------
% Example: Quadratic Curve Fit of Distance vs. Cy in MATLAB
% --------------------------------------------------------------

% 1) Read data from your CSV file.
%    Suppose your CSV has the following columns (just as an example):
%       Column 1: index or other info
%       Column 2: Cy (vertical coordinate of blob)
%       Column 3: Distance (cm)
%    Adjust 'mydata.csv' and the column indices as needed.

% data = readmatrix('stop_distance_calc.csv');  % or use csvread if older MATLAB versions
% cy = data(2:end, 4);      % Cy column
% dist = data(2:end, 12);    % Distance column
% cy   = [0, 18, 65, 117, 197, 323, 480];
% dist = [33.45, 30.3, 23.5, 17.65, 11.85, 6, 0] + 6.9;
% cy = [28, 73, 136, 232, 395];
% dist = [25.7, 19.9, 14.05, 8.2, 2.35] + 7.9;

% cy = [0, 25, 57, 106, 188, 480];
% dist = [28.95, 22, 16.15, 10.3, 4.45, 0] + 5.8;

cy = [0, 4, 35, 81, 155, 480];
dist = [22.5, 20.15, 14.3, 8.5, 2.65, 0] + 7.5;

method = 'poly'; % poly, exponential, log
power = 4;

%% 2) Fit a quadratic polynomial:
%    dist = p(1)*cy^2 + p(2)*cy + p(3)
if strcmp(method, 'poly')
    p = polyfit(cy, dist, power);  % # indicates the degree (quadratic, cubic, etc) fit
    
    % 3) Evaluate the fitted polynomial at the same Cy values
    fittedDist = polyval(p, cy);
    disp(p)
    
    % 4) Plot the raw data and the fitted curve
    figure;
    plot(cy, dist, 'bo', 'MarkerFaceColor', 'b');  % Blue dots for raw data
    hold on;
    plot(cy, fittedDist, 'r-', 'LineWidth', 2);    % Red line for fitted curve
    xlabel('Cy (pixels)');
    ylabel('Distance (cm)');
    legend('Data',['Polynomial ' num2str(power) ' Fit'],'Location','best');
    title(['Polynomial ' num2str(power) ' Fit of Distance vs. Cy']);
    
    % 5) Display the fitted equation in the Command Window
    disp(['Fitted ' num2str(power) ' polynomial:']);
    % disp(['Distance = ' num2str(p(1)) ' * blob.cy()**4 + ' ...
    %                    num2str(p(2)) ' * blob.cy()**3 + ' ...
    %                    num2str(p(3)) ' * blob.cy()**2 + ' ...
    %                    num2str(p(4)) ' * blob.cy() + ' ...
    %                    num2str(p(5))]);
    disp('Distance =')
    to_the = length(p);
    for i = 1 : length(p)
        to_the = to_the - 1;
        disp([num2str(p(i)) ' * y**' num2str(to_the)])
    end
    
    % 6) (Optional) Compute R-squared to assess fit quality
    residuals = dist - fittedDist;
    SSresid   = sum(residuals.^2);
    SStotal   = (length(dist) - 1) * var(dist);
    Rsq       = 1 - SSresid / SStotal;
    disp(['R-squared = ', num2str(Rsq)]);
end

%% Non linear - exponential

if strcmp(method, 'exponential')
% 2) Define the exponential model: Distance = a * exp(b * Cy)
model = @(params, x) params(1) * exp(params(2) * x);

% Define the cost function (sum of squared errors)
costFunction = @(params) sum((model(params, cy) - dist).^2);

% Provide an initial guess for [a, b]
initial_guess = [mean(dist), 0.001];

% Use fminsearch to find the parameters that minimize the cost function
opt_params = fminsearch(costFunction, initial_guess);
a = opt_params(1);
b = opt_params(2);

% Display the fitted exponential model
fprintf('Fitted exponential model using fminsearch: Distance = %f * math.exp(%f * blob.cy())\n', a, b);

% 3) Calculate the predicted distances and R-squared
predictedDist = model(opt_params, cy);
residuals = dist - predictedDist;
SSresid = sum(residuals.^2);
SStotal = sum((dist - mean(dist)).^2);
Rsq = 1 - SSresid / SStotal;
fprintf('R-squared: %f\n', Rsq);

% 4) Plot the data and the fitted curve
cy_fit = linspace(min(cy), max(cy), 100);
dist_fit = model(opt_params, cy_fit);

figure;
plot(cy, dist, 'bo', 'MarkerFaceColor', 'b');  % Data points
hold on;
plot(cy_fit, dist_fit, 'r-', 'LineWidth', 2);    % Fitted curve
xlabel('Cy (pixels)');
ylabel('Distance (cm)');
legend('Data', 'Exponential Fit', 'Location', 'best');
title('Exponential Fit of Distance vs. Cy (using fminsearch)');
end

%% Log

if strcmp(method, 'log')
    % Ensure all distance values are positive (required for logarithm)
    if any(dist <= 0)
        error('All distance values must be positive for the logarithm transform.');
    end
    
    % 2) Linearize the model by taking the natural logarithm of the distance data
    logDist = log(dist);
    
    % Fit a linear model: log(Distance) = b * Cy + log(a)
    p = polyfit(cy, logDist, 1);  % p(1) is b, p(2) is log(a)
    b = p(1);
    a = exp(p(2));
    
    % Display the fitted exponential model
    fprintf('Fitted exponential model: Distance = %f * math.exp(%f * blob.cy())\n', a, b);
    
    % 3) Calculate the predicted distances and R-squared on the original scale
    predictedDist = a * exp(b * cy);
    residuals = dist - predictedDist;
    SSresid = sum(residuals.^2);
    SStotal = sum((dist - mean(dist)).^2);
    Rsq = 1 - SSresid / SStotal;
    fprintf('R-squared: %f\n', Rsq);
    
    % 4) Plot the original data and the fitted exponential curve
    cy_fit = linspace(min(cy), max(cy), 100);
    dist_fit = a * exp(b * cy_fit);
    
    figure;
    plot(cy, dist, 'bo', 'MarkerFaceColor', 'b');  % Raw data points
    hold on;
    plot(cy_fit, dist_fit, 'r-', 'LineWidth', 2);    % Fitted exponential curve
    xlabel('Cy (pixels)');
    ylabel('Distance (cm)');
    legend('Data','Exponential Fit','Location','best');
    title('Exponential Fit of Distance vs. Cy (using log transformation)');
end

