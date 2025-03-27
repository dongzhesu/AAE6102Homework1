function plotEKFNavigation(navSolutions, ekfSolutions, settings)
% Function plots the navigation results: the receiver position in geodetic
% coordinates and North, East, Up format. It also plots the receiver
% clock drift error. In each figure, both the WLS and EKF solutions are plotted
% for comparison.
%
% plotEKFNavigation(navSolutions, ekfSolutions, settings)
%
%   Inputs:
%       navSolutions    - contains the original WLS navigation solutions
%       ekfSolutions    - contains the EKF navigation solutions
%       settings        - receiver settings

%% Check if some essential parameters exist in the settings
if ~isfield(settings, 'navSolPeriod')
    settings.navSolPeriod = 500; % Default period in ms
end

%% Plot all figures =======================================================
% Use some predefined parameters for the figures
figPosition = [50, 50, 1200, 600];
NColor = [0, 0.5, 0]; % Dark green for North
EColor = [0.8, 0, 0]; % Dark red for East
UColor = [0, 0, 0.8]; % Dark blue for Up

% Compute local time vector for plotting
if isfield(navSolutions, 'localTime')
    timeAxisSeconds = navSolutions.localTime;
else
    % If no local time exists, create a time vector based on navigation period
    timeAxisSeconds = (0:(length(navSolutions.X)-1)) * settings.navSolPeriod / 1000;
end

%% Plot receiver position in geodetic coordinates =======================
figure
set(gcf, 'Position', figPosition);
subplot(3, 1, 1)
plot(timeAxisSeconds, navSolutions.latitude, 'b.', 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.latitude, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Latitude (WLS blue dots, EKF red line)');
xlabel('Time (s)');
ylabel('Degrees');
legend('WLS', 'EKF');

subplot(3, 1, 2)
plot(timeAxisSeconds, navSolutions.longitude, 'b.', 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.longitude, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Longitude (WLS blue dots, EKF red line)');
xlabel('Time (s)');
ylabel('Degrees');
legend('WLS', 'EKF');

subplot(3, 1, 3)
plot(timeAxisSeconds, navSolutions.height, 'b.', 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.height, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Height (WLS blue dots, EKF red line)');
xlabel('Time (s)');
ylabel('m');
legend('WLS', 'EKF');

%% Plot receiver position in UTM coordinates (North, East, Up) =============
figure
set(gcf, 'Position', figPosition);

% North plot
subplot(3, 1, 1)
plot(timeAxisSeconds, navSolutions.N, '.', 'Color', NColor, 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.N, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('North (WLS green dots, EKF red line)');
xlabel('Time (s)');
ylabel('m');
legend('WLS', 'EKF');

% East plot
subplot(3, 1, 2)
plot(timeAxisSeconds, navSolutions.E, '.', 'Color', EColor, 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.E, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('East (WLS red dots, EKF red line)');
xlabel('Time (s)');
ylabel('m');
legend('WLS', 'EKF');

% Up plot
subplot(3, 1, 3)
plot(timeAxisSeconds, navSolutions.U, '.', 'Color', UColor, 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.U, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Up (WLS blue dots, EKF red line)');
xlabel('Time (s)');
ylabel('m');
legend('WLS', 'EKF');

%% Plot clock drift error ================================================
figure
set(gcf, 'Position', figPosition);
subplot(2, 1, 1)
plot(timeAxisSeconds, navSolutions.dt, 'b.', 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.dt, 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Clock Bias (WLS blue dots, EKF red line)');
xlabel('Time (s)');
ylabel('m');
legend('WLS', 'EKF');

subplot(2, 1, 2)
% Instead of pseudorange residuals which may not be available, plot DOPs
% Extract DOPs
GDOP = ekfSolutions.DOP(1, :);
PDOP = ekfSolutions.DOP(2, :);
HDOP = ekfSolutions.DOP(3, :);
VDOP = ekfSolutions.DOP(4, :);
TDOP = ekfSolutions.DOP(5, :);

% Plot DOPs
plot(timeAxisSeconds, GDOP, 'k-', 'LineWidth', 2);
hold on;
plot(timeAxisSeconds, PDOP, 'b-', 'LineWidth', 2);
plot(timeAxisSeconds, HDOP, 'g-', 'LineWidth', 2);
plot(timeAxisSeconds, VDOP, 'r-', 'LineWidth', 2);
plot(timeAxisSeconds, TDOP, 'm-', 'LineWidth', 2);
hold off;
grid on;
title('Dilution of Precision (EKF solution)');
xlabel('Time (s)');
ylabel('DOP');
legend('GDOP', 'PDOP', 'HDOP', 'VDOP', 'TDOP');

%% Plot horizontal position scatter plot ==================================
figure
set(gcf, 'Position', figPosition);

% Extract valid coordinates
validIdx = ~isnan(navSolutions.E) & ~isnan(navSolutions.N) & ...
           ~isnan(ekfSolutions.E) & ~isnan(ekfSolutions.N);

% Calculate mean values for centering
meanE_wls = mean(navSolutions.E(validIdx));
meanN_wls = mean(navSolutions.N(validIdx));
meanE_ekf = mean(ekfSolutions.E(validIdx));
meanN_ekf = mean(ekfSolutions.N(validIdx));

% Plot centered positions
subplot(1, 2, 1)
scatter(navSolutions.E(validIdx) - meanE_wls, navSolutions.N(validIdx) - meanN_wls, 15, 'b', 'filled');
grid on;
axis equal;
title('WLS Position Scatter (centered)');
xlabel('East (m)');
ylabel('North (m)');

subplot(1, 2, 2)
scatter(ekfSolutions.E(validIdx) - meanE_ekf, ekfSolutions.N(validIdx) - meanN_ekf, 15, 'r', 'filled');
grid on;
axis equal;
title('EKF Position Scatter (centered)');
xlabel('East (m)');
ylabel('North (m)');

%% Plot number of satellites and GDOP ====================================
figure
set(gcf, 'Position', figPosition);

% Number of satellites
subplot(2, 1, 1)
% Count valid satellites at each time step for WLS solution
numSatsWLS = sum(~isnan(navSolutions.PRN), 1);
plot(timeAxisSeconds, numSatsWLS, 'b.-', 'LineWidth', 1, 'MarkerSize', 10);
hold on;
% Count valid satellites at each time step for EKF solution
if isfield(ekfSolutions, 'numSatellites')
    numSatsEKF = ekfSolutions.numSatellites;
    plot(timeAxisSeconds, numSatsEKF, 'r-', 'LineWidth', 2);
end
hold off;
grid on;
title('Number of Satellites Used');
xlabel('Time (s)');
ylabel('Count');
legend('WLS', 'EKF');

% GDOP
subplot(2, 1, 2)
plot(timeAxisSeconds, navSolutions.DOP(1, :), 'b.-', 'LineWidth', 1, 'MarkerSize', 10);
hold on;
plot(timeAxisSeconds, ekfSolutions.DOP(1, :), 'r-', 'LineWidth', 2);
hold off;
grid on;
title('Geometric Dilution of Precision (GDOP)');
xlabel('Time (s)');
ylabel('GDOP');
legend('WLS', 'EKF');

%% Plot trajectory on a 2D map if ground truth is available =============
% If ground truth is defined, we'll plot comparison with ground truth
if isfield(settings, 'groundTruth') && ~isempty(settings.groundTruth)
    figure
    set(gcf, 'Position', figPosition);
    
    % Plot ground truth
    plot(settings.groundTruth.longitude, settings.groundTruth.latitude, 'k-', 'LineWidth', 3);
    hold on;
    
    % Plot WLS solution
    plot(navSolutions.longitude, navSolutions.latitude, 'b.', 'MarkerSize', 10);
    
    % Plot EKF solution
    plot(ekfSolutions.longitude, ekfSolutions.latitude, 'r-', 'LineWidth', 2);
    
    grid on;
    title('Trajectory Comparison');
    xlabel('Longitude (degrees)');
    ylabel('Latitude (degrees)');
    legend('Ground Truth', 'WLS', 'EKF');
    hold off;
end

%% Print statistics =====================================================
% Calculate position error statistics
validIdx = ~isnan(navSolutions.E) & ~isnan(navSolutions.N) & ~isnan(navSolutions.U) & ...
           ~isnan(ekfSolutions.E) & ~isnan(ekfSolutions.N) & ~isnan(ekfSolutions.U);

% Calculate standard deviations
stdE_wls = std(navSolutions.E(validIdx));
stdN_wls = std(navSolutions.N(validIdx));
stdU_wls = std(navSolutions.U(validIdx));
stdE_ekf = std(ekfSolutions.E(validIdx));
stdN_ekf = std(ekfSolutions.N(validIdx));
stdU_ekf = std(ekfSolutions.U(validIdx));

% Calculate horizontal and 3D position error standard deviations
stdHorizontal_wls = sqrt(stdE_wls^2 + stdN_wls^2);
std3D_wls = sqrt(stdE_wls^2 + stdN_wls^2 + stdU_wls^2);
stdHorizontal_ekf = sqrt(stdE_ekf^2 + stdN_ekf^2);
std3D_ekf = sqrt(stdE_ekf^2 + stdN_ekf^2 + stdU_ekf^2);

% Print statistics to console
fprintf('\n===================== Position Error Statistics =====================\n');
fprintf('                    WLS Solution      EKF Solution     Improvement\n');
fprintf('East Std Dev:      %8.3f m       %8.3f m       %7.2f%%\n', ...
        stdE_wls, stdE_ekf, 100*(1 - stdE_ekf/stdE_wls));
fprintf('North Std Dev:     %8.3f m       %8.3f m       %7.2f%%\n', ...
        stdN_wls, stdN_ekf, 100*(1 - stdN_ekf/stdN_wls));
fprintf('Up Std Dev:        %8.3f m       %8.3f m       %7.2f%%\n', ...
        stdU_wls, stdU_ekf, 100*(1 - stdU_ekf/stdU_wls));
fprintf('Horizontal Std:    %8.3f m       %8.3f m       %7.2f%%\n', ...
        stdHorizontal_wls, stdHorizontal_ekf, 100*(1 - stdHorizontal_ekf/stdHorizontal_wls));
fprintf('3D Position Std:   %8.3f m       %8.3f m       %7.2f%%\n', ...
        std3D_wls, std3D_ekf, 100*(1 - std3D_ekf/std3D_wls));
fprintf('====================================================================\n\n');

% Create figure with position error statistics
figure
set(gcf, 'Position', [50, 50, 800, 400]);

statsData = [stdE_wls, stdE_ekf; ...
             stdN_wls, stdN_ekf; ...
             stdU_wls, stdU_ekf; ...
             stdHorizontal_wls, stdHorizontal_ekf; ...
             std3D_wls, std3D_ekf];

bar(statsData);
grid on;
title('Position Error Standard Deviation Comparison');
xlabel('Error Component');
ylabel('Standard Deviation (m)');
set(gca, 'XTickLabel', {'East', 'North', 'Up', 'Horizontal', '3D'});
legend('WLS', 'EKF');

% Add improvement percentages as text annotations
for i = 1:5
    improvement = 100*(1 - statsData(i,2)/statsData(i,1));
    text(i, statsData(i,2) + 0.05*max(statsData(:)), ...
         [num2str(improvement, '%.1f') '%'], ...
         'HorizontalAlignment', 'center');
end

end
