function ekfSolutions = applyEKF(navSolutions, trackResults, settings, eph)
% Applies Extended Kalman Filter to estimate user position using pseudoranges
% and Doppler measurements.
%
% Inputs:
%   navSolutions - structure containing WLS position solutions, pseudoranges,
%                  satellite positions, etc.
%   trackResults - structure containing tracking results
%   settings     - receiver configuration parameters
%
% Outputs:
%   ekfSolutions - structure containing EKF position solutions with fields
%                  similar to navSolutions for comparison

%% Initialize EKF parameters
% Speed of light
c = settings.c;  % [m/s]
disp('Starting EKF initialization...');

% Number of measurement points (epochs)
numMeasurements = length(navSolutions.X);
disp(['Total number of measurement epochs: ', num2str(numMeasurements)]);

% Skip the measurement points where position could not be calculated
validMeas = ~isnan(navSolutions.X);
numValidMeasurements = sum(validMeas);
disp(['Number of valid measurement epochs: ', num2str(numValidMeasurements)]);

% If there are no valid measurements, return empty structure
if numValidMeasurements == 0
    ekfSolutions = [];
    disp('ERROR: No valid measurements found. EKF cannot be applied.');
    return;
end

% State vector: [x, y, z, dt]
% x, y, z: User position in ECEF coordinates (meters)
% dt: Receiver clock bias (meters)
numStates = 4;
disp(['EKF state vector dimensions: ', num2str(numStates)]);

% Initialize state vector with first valid WLS solution
firstValidIdx = find(validMeas, 1);
x = [navSolutions.X(firstValidIdx); 
     navSolutions.Y(firstValidIdx); 
     navSolutions.Z(firstValidIdx); 
     navSolutions.dt(firstValidIdx)];
disp('Initial state vector:');
disp(['X: ', num2str(x(1))]);
disp(['Y: ', num2str(x(2))]);
disp(['Z: ', num2str(x(3))]);
disp(['dt: ', num2str(x(4))]);

% Initialize state covariance matrix
% Initial uncertainty in position (meters)
initialPosUncertainty = 10.0;
% Initial uncertainty in clock bias (meters)
initialClkUncertainty = 3000.0;
P = diag([initialPosUncertainty^2, initialPosUncertainty^2, initialPosUncertainty^2, initialClkUncertainty^2]);
disp('Initial state covariance matrix diagonal:');
disp(diag(P)');

% Process noise covariance matrix Q
% Position states process noise (m^2)
posNoise = 1.0;
% Clock bias process noise (m^2)
clockBiasNoise = 1000.0;
Q = diag([posNoise, posNoise, posNoise, clockBiasNoise]);
disp('Process noise covariance matrix Q diagonal:');
disp(diag(Q)');

% Measurement noise covariance matrix R
% Pseudorange measurement noise (m^2)
prNoise = 25.0;
disp(['Pseudorange measurement noise: ', num2str(prNoise), ' m^2']);

% State transition matrix (static position model, random walk clock bias)
Phi = eye(numStates);
disp('State transition matrix is identity (static position model)');

% Pre-allocate output structure fields
disp('Pre-allocating output structure fields...');
ekfSolutions.X = nan(1, numMeasurements);
ekfSolutions.Y = nan(1, numMeasurements);
ekfSolutions.Z = nan(1, numMeasurements);
ekfSolutions.dt = nan(1, numMeasurements);
ekfSolutions.latitude = nan(1, numMeasurements);
ekfSolutions.longitude = nan(1, numMeasurements);
ekfSolutions.height = nan(1, numMeasurements);
ekfSolutions.E = nan(1, numMeasurements);
ekfSolutions.N = nan(1, numMeasurements);
ekfSolutions.U = nan(1, numMeasurements);
ekfSolutions.DOP = nan(5, numMeasurements);
ekfSolutions.numSatellites = nan(1, numMeasurements);
ekfSolutions.covMatrix = cell(1, numMeasurements);

% Copy other fields from navSolutions for consistency
ekfSolutions.localTime = navSolutions.localTime;
ekfSolutions.currMeasSample = navSolutions.currMeasSample;
ekfSolutions.el = navSolutions.el;
ekfSolutions.az = navSolutions.az;
ekfSolutions.PRN = navSolutions.PRN;
ekfSolutions.rawP = navSolutions.rawP;
ekfSolutions.correctedP = navSolutions.correctedP;
ekfSolutions.transmitTime = navSolutions.transmitTime;
ekfSolutions.satClkCorr = navSolutions.satClkCorr;
ekfSolutions.satPositions = cell(1, numMeasurements);
disp('Output structure initialized successfully');

%% Process each measurement epoch
disp('Beginning EKF processing for each measurement epoch...');

for i = 1:numMeasurements
    disp(['Processing epoch ', num2str(i), ' of ', num2str(numMeasurements), '...']);
    
    % Skip if no valid measurement at this epoch
    if ~validMeas(i)
        disp(['Epoch ', num2str(i), ': No valid WLS solution, skipping...']);
        continue;
    end
    
    % Get PRN numbers of visible satellites at this epoch
    availablePRNs = navSolutions.PRN(:, i);
    availablePRNs = availablePRNs(~isnan(availablePRNs));
    
    % Important fix: Only use the PRNs that exist in the availablePRNs array
    numVisibleSats = length(availablePRNs);
    
    disp(['Epoch ', num2str(i), ': Number of visible satellites: ', num2str(numVisibleSats)]);
    if numVisibleSats > 0
        disp(['Visible PRNs: ', num2str(availablePRNs')]);
    else
        disp('No visible satellites detected.');
    end
    
    % Skip if not enough satellites
    if numVisibleSats < 4
        disp(['Epoch ', num2str(i), ': Not enough satellites (< 4), skipping...']);
        continue;
    end
    
    % Time update (prediction)
    disp('Performing time update (prediction)...');
    x_pred = Phi * x;
    P_pred = Phi * P * Phi' + Q;
    
    disp('Predicted state:');
    disp(['X_pred: ', num2str(x_pred(1))]);
    disp(['Y_pred: ', num2str(x_pred(2))]);
    disp(['Z_pred: ', num2str(x_pred(3))]);
    disp(['dt_pred: ', num2str(x_pred(4))]);
    
    % Get pseudoranges for visible satellites
    % Create a logical index to select only the available PRNs from raw pseudoranges
    prnIndices = zeros(size(navSolutions.rawP, 1), 1);
    for j = 1:numVisibleSats
        prnIdx = find(navSolutions.PRN(:, i) == availablePRNs(j), 1);
        if ~isempty(prnIdx)
            prnIndices(prnIdx) = 1;
        end
    end
    prnIndices = logical(prnIndices);
    
    % Get pseudorange measurements for available satellites
    z = navSolutions.rawP(prnIndices, i);
    
    disp('Raw pseudorange measurements:');
    disp(z);
    
    % Check for NaN or inf values in pseudoranges
    if any(isnan(z)) || any(isinf(z))
        disp('WARNING: NaN or Inf values in pseudorange measurements!');
        nanIdx = find(isnan(z));
        infIdx = find(isinf(z));
        if ~isempty(nanIdx)
            disp('NaN indices:');
            disp(nanIdx);
        end
        if ~isempty(infIdx)
            disp('Inf indices:');
            disp(infIdx);
        end
        
        % Remove NaN or Inf values
        validZ = ~isnan(z) & ~isinf(z);
        z = z(validZ);
        
        % Update available PRNs accordingly
        availablePRNs = availablePRNs(validZ);
        numVisibleSats = length(availablePRNs);
        
        if numVisibleSats < 4
            disp(['After removing NaN/Inf values, not enough valid satellites (', num2str(numVisibleSats), ' < 4). Skipping epoch.']);
            continue;
        end
    end
    
    % Correct pseudorange for satellite clock errors
    satClkCorrValues = zeros(numVisibleSats, 1);
    for j = 1:numVisibleSats
        prnIdx = find(navSolutions.PRN(:, i) == availablePRNs(j), 1);
        if ~isempty(prnIdx) && ~isnan(navSolutions.satClkCorr(prnIdx, i))
            satClkCorrValues(j) = navSolutions.satClkCorr(prnIdx, i);
        end
    end
    
    if ~all(satClkCorrValues == 0)
        z = z + satClkCorrValues * c;
        disp('Applied satellite clock corrections to pseudoranges');
    else
        disp('WARNING: No satellite clock corrections available or all zero');
    end
    
    % Prepare for satellite position calculation
    satPositions = zeros(numVisibleSats, 3);
    validSats = true(numVisibleSats, 1);
    
    % Calculate satellite positions
    disp('Calculating satellite positions...');
    
    % Import ephemeris data - Make sure 'eph' is available!
    % This should be loaded or passed from the main script
    eph = evalin('base', 'eph');
    
    for j = 1:numVisibleSats
        prn = availablePRNs(j);
        satIdx = find(navSolutions.PRN(:, i) == prn, 1);
        
        if ~isempty(satIdx) && ~isnan(navSolutions.transmitTime(satIdx, i))
            try
                % Create a transmitTime array with one entry for this PRN
                % The satpos function expects an array of times, one per PRN
                timeArray = navSolutions.transmitTime(satIdx, i);
                
                % Call satpos with the correct signature
                [satPos, satClkCorr] = satpos(timeArray, prn, eph);
                
                % satPos is a 3x1 matrix containing [X; Y; Z] for this satellite
                satPositions(j, :) = satPos';  % Transpose to get [X, Y, Z] row vector
                
                disp(['Satellite PRN ', num2str(prn), ' position calculated:']);
                disp(['X: ', num2str(satPos(1)), ', Y: ', num2str(satPos(2)), ', Z: ', num2str(satPos(3))]);
            catch ME
                disp(['WARNING: Error calculating position for PRN ', num2str(prn), ': ', ME.message]);
                validSats(j) = false;
            end
        else
            disp(['WARNING: No valid transmission time for PRN ', num2str(prn)]);
            validSats(j) = false;
        end
    end
    
    % Remove satellites with invalid positions
    if ~all(validSats)
        disp(['Removing ', num2str(sum(~validSats)), ' satellites with invalid positions.']);
        satPositions = satPositions(validSats, :);
        z = z(validSats);
        availablePRNs = availablePRNs(validSats);
        numVisibleSats = length(availablePRNs);
        
        if numVisibleSats < 4
            disp(['Not enough satellites with valid positions (', num2str(numVisibleSats), ' < 4). Skipping epoch.']);
            continue;
        end
    end
    
    % Store satellite positions for this epoch
    ekfSolutions.satPositions{i} = satPositions;
    
    % Calculate predicted measurements (predicted pseudoranges)
    h = zeros(numVisibleSats, 1);
    H = zeros(numVisibleSats, numStates);
    
    disp('Calculating predicted measurements and Jacobian matrix...');
    
    for j = 1:numVisibleSats
        % Calculate distance between satellite and predicted position
        dx = satPositions(j, 1) - x_pred(1);
        dy = satPositions(j, 2) - x_pred(2);
        dz = satPositions(j, 3) - x_pred(3);
        
        % Predicted pseudorange
        dist = sqrt(dx^2 + dy^2 + dz^2);
        
        % Check for unrealistic distances
        if dist < 1000 || dist > 100000000
            disp(['WARNING: Unrealistic distance to satellite PRN ', num2str(availablePRNs(j)), ': ', num2str(dist), ' meters']);
        end
        
        h(j) = dist + x_pred(4);
        
        % Jacobian of measurement model w.r.t. states
        H(j, 1) = -dx / dist;  % Partial derivative w.r.t. x
        H(j, 2) = -dy / dist;  % Partial derivative w.r.t. y
        H(j, 3) = -dz / dist;  % Partial derivative w.r.t. z
        H(j, 4) = 1;           % Partial derivative w.r.t. dt
    end
    
    disp('Predicted measurements (pseudoranges):');
    disp(h);
    
    disp('Jacobian matrix H (first few rows):');
    if numVisibleSats > 3
        disp(H(1:min(3, numVisibleSats), :));
    else
        disp(H);
    end
    
    % Measurement noise covariance scaled by satellite elevations
    R = eye(numVisibleSats) * prNoise;
    
    % Apply elevation-dependent weighting if elevation data is available
    disp('Applying elevation-dependent weighting to measurement noise...');
    for j = 1:numVisibleSats
        satIdx = find(navSolutions.PRN(:, i) == availablePRNs(j), 1);
        if ~isempty(satIdx) && ~isnan(navSolutions.el(satIdx, i))
            % Scale noise inversely proportional to sin of elevation angle
            elev_deg = navSolutions.el(satIdx, i);
            elev_rad = max(elev_deg, 5) * pi/180;  % Minimum elevation of 5 degrees
            R(j, j) = prNoise / (sin(elev_rad))^2;
            
            disp(['PRN ', num2str(availablePRNs(j)), ', Elevation: ', num2str(elev_deg), ...
                  ' deg, R: ', num2str(R(j, j)), ' m^2']);
        else
            disp(['WARNING: No elevation data for PRN ', num2str(availablePRNs(j)), ...
                  ', using default noise: ', num2str(R(j, j)), ' m^2']);
        end
    end
    
    % Calculate Kalman gain
    disp('Calculating innovation covariance and Kalman gain...');
    S = H * P_pred * H' + R;
    
    % Check for singularity in S
    if rcond(S) < 1e-10
        disp('WARNING: Innovation covariance matrix S is nearly singular!');
        disp(['Condition number: ', num2str(rcond(S))]);
        % Use pseudo-inverse instead
        disp('Using pseudoinverse for S');
        K = P_pred * H' * pinv(S);
    else
        K = P_pred * H' / S;
    end
    
    % Measurement update (correction)
    innovation = z - h;
    disp('Innovation vector:');
    disp(innovation);
    
    % Large innovation check to detect outliers
    maxInnovation = 100; % meters
    validMeasIdx = abs(innovation) < maxInnovation;
    
    if sum(~validMeasIdx) > 0
        disp(['WARNING: ', num2str(sum(~validMeasIdx)), ' large innovations detected!']);
        disp(['  PRNs with large innovations: ', num2str(availablePRNs(~validMeasIdx)')]);
        disp(['  Innovation values: ', num2str(innovation(~validMeasIdx)')]);
    end
    
    if sum(validMeasIdx) >= 4
        disp(['Using ', num2str(sum(validMeasIdx)), ' valid measurements for update.']);
        
        % Use only valid measurements for update
        H_valid = H(validMeasIdx, :);
        innovation_valid = innovation(validMeasIdx);
        R_valid = R(validMeasIdx, validMeasIdx);
        
        % Recalculate Kalman gain with valid measurements
        S_valid = H_valid * P_pred * H_valid' + R_valid;
        
        % Check for singularity in S_valid
        if rcond(S_valid) < 1e-10
            disp('WARNING: Valid innovation covariance matrix S_valid is nearly singular!');
            disp(['Condition number: ', num2str(rcond(S_valid))]);
            % Use pseudo-inverse instead
            disp('Using pseudoinverse for S_valid');
            K_valid = P_pred * H_valid' * pinv(S_valid);
        else
            K_valid = P_pred * H_valid' / S_valid;
        end
        
        % Update state and covariance
        state_correction = K_valid * innovation_valid;
        disp('State correction:');
        disp(['  dX: ', num2str(state_correction(1))]);
        disp(['  dY: ', num2str(state_correction(2))]);
        disp(['  dZ: ', num2str(state_correction(3))]);
        disp(['  d(dt): ', num2str(state_correction(4))]);
        
        x = x_pred + state_correction;
        P = (eye(numStates) - K_valid * H_valid) * P_pred;
    else
        disp(['WARNING: Not enough valid measurements (', num2str(sum(validMeasIdx)), ' < 4), using prediction.']);
        % Not enough valid measurements, use prediction
        x = x_pred;
        P = P_pred;
    end
    
    disp('Updated state:');
    disp(['X: ', num2str(x(1))]);
    disp(['Y: ', num2str(x(2))]);
    disp(['Z: ', num2str(x(3))]);
    disp(['dt: ', num2str(x(4))]);
    
    % Check for NaN or Inf in the solution
    if any(isnan(x)) || any(isinf(x))
        disp('ERROR: NaN or Inf values in the state estimate!');
        disp('Using prediction instead.');
        x = x_pred;
        P = P_pred;
    end
    
    % Store the EKF solution
    ekfSolutions.X(i) = x(1);
    ekfSolutions.Y(i) = x(2);
    ekfSolutions.Z(i) = x(3);
    ekfSolutions.dt(i) = x(4);
    ekfSolutions.covMatrix{i} = P;
    ekfSolutions.numSatellites(i) = sum(validMeasIdx);
    
    % Convert ECEF to geodetic coordinates (lat, lon, height)
    disp('Converting ECEF to geodetic coordinates...');
    try
        [lat, lon, h] = cart2geo(x(1), x(2), x(3), 5);
        disp(['Latitude: ', num2str(lat), ' deg']);
        disp(['Longitude: ', num2str(lon), ' deg']);
        disp(['Height: ', num2str(h), ' m']);
        
        ekfSolutions.latitude(i) = lat;
        ekfSolutions.longitude(i) = lon;
        ekfSolutions.height(i) = h;
    catch ME
        disp(['ERROR in cart2geo conversion: ', ME.message]);
        ekfSolutions.latitude(i) = NaN;
        ekfSolutions.longitude(i) = NaN;
        ekfSolutions.height(i) = NaN;
    end
    
    % Convert to UTM coordinates
    disp('Converting to UTM coordinates...');
    try
        utmZone = findUtmZone(lat, lon);
        ekfSolutions.utmZone = utmZone;
        disp(['UTM Zone: ', utmZone]);
        
        [E, N, U] = cart2utm(x(1), x(2), x(3), utmZone);
        disp(['E: ', num2str(E), ' m']);
        disp(['N: ', num2str(N), ' m']);
        disp(['U: ', num2str(U), ' m']);
        
        ekfSolutions.E(i) = E;
        ekfSolutions.N(i) = N;
        ekfSolutions.U(i) = U;
    catch ME
        disp(['ERROR in UTM conversion: ', ME.message]);
        ekfSolutions.E(i) = NaN;
        ekfSolutions.N(i) = NaN;
        ekfSolutions.U(i) = NaN;
    end
    
    % Calculate Dilution of Precision (DOP) values
    % Only if we have valid measurements
    disp('Calculating DOP values...');
    if sum(validMeasIdx) >= 4
        H_valid = H(validMeasIdx, :);
        % Calculate G matrix (normalized line of sight vectors + 1 for clock)
        G = H_valid;
        
        % Calculate DOP matrix
        if rank(G) >= 4
            try
                DOP_matrix = inv(G' * G);
                
                % GDOP - Geometric DOP
                ekfSolutions.DOP(1, i) = sqrt(trace(DOP_matrix));
                % PDOP - Position DOP
                ekfSolutions.DOP(2, i) = sqrt(DOP_matrix(1,1) + DOP_matrix(2,2) + DOP_matrix(3,3));
                % HDOP - Horizontal DOP
                ekfSolutions.DOP(3, i) = sqrt(DOP_matrix(1,1) + DOP_matrix(2,2));
                % VDOP - Vertical DOP
                ekfSolutions.DOP(4, i) = sqrt(DOP_matrix(3,3));
                % TDOP - Time DOP
                ekfSolutions.DOP(5, i) = sqrt(DOP_matrix(4,4));
                
                disp(['GDOP: ', num2str(ekfSolutions.DOP(1, i))]);
                disp(['PDOP: ', num2str(ekfSolutions.DOP(2, i))]);
                disp(['HDOP: ', num2str(ekfSolutions.DOP(3, i))]);
                disp(['VDOP: ', num2str(ekfSolutions.DOP(4, i))]);
                disp(['TDOP: ', num2str(ekfSolutions.DOP(5, i))]);
            catch ME
                disp(['ERROR in DOP calculation: ', ME.message]);
                ekfSolutions.DOP(:, i) = nan(5, 1);
            end
        else
            disp(['WARNING: Not enough rank in G matrix: ', num2str(rank(G)), ' < 4']);
            ekfSolutions.DOP(:, i) = nan(5, 1);
        end
    else
        disp('Not enough valid measurements for DOP calculation');
        ekfSolutions.DOP(:, i) = nan(5, 1);
    end
    
    disp(['Epoch ', num2str(i), ' processing complete']);
    disp('---------------------------------------------------');
end

% Add a description field to identify this solution method
ekfSolutions.description = 'Extended Kalman Filter solution';
disp('EKF processing completed for all epochs');

end

% Helper function to convert from ECEF to geodetic coordinates
function [latitude, longitude, height] = cart2geo(X, Y, Z, iterations)
% World Geodetic System 1984 parameters
a = 6378137.0;             % semi-major axis [m]
f = 1/298.257223563;       % flattening
b = a*(1-f);               % semi-minor axis [m]
e = sqrt(2*f - f^2);       % first eccentricity
ep = e / sqrt(1 - e^2);    % second eccentricity

disp('cart2geo: Converting ECEF to geodetic coordinates');
disp(['Input ECEF - X: ', num2str(X), ', Y: ', num2str(Y), ', Z: ', num2str(Z)]);

% Check for invalid input
if isnan(X) || isnan(Y) || isnan(Z) || isinf(X) || isinf(Y) || isinf(Z)
    disp('ERROR: Invalid ECEF coordinates (NaN or Inf)');
    latitude = NaN;
    longitude = NaN;
    height = NaN;
    return;
end

% Longitude
longitude = atan2(Y, X);

% Initialize values for latitude and height calculation
p = sqrt(X^2 + Y^2);
if p < 1e-10
    disp('WARNING: Very small p value, position is near poles');
    % Handle special case for poles
    if Z > 0
        latitude = pi/2;  % North pole
    else
        latitude = -pi/2;  % South pole
    end
    height = abs(Z) - b;
    
    % Convert to degrees
    latitude = latitude * 180 / pi;
    longitude = longitude * 180 / pi;
    
    disp(['Output - Latitude: ', num2str(latitude), ', Longitude: ', num2str(longitude), ', Height: ', num2str(height)]);
    return;
end

lat = atan2(Z, p * (1 - e^2));
N = a / sqrt(1 - e^2 * sin(lat)^2);
h = p / cos(lat) - N;

disp('Starting iterative latitude calculation');
% Iteratively calculate latitude and height
for i = 1:iterations
    lat_prev = lat;
    N = a / sqrt(1 - e^2 * sin(lat)^2);
    h = p / cos(lat) - N;
    lat = atan2(Z, p * (1 - e^2 * N / (N + h)));
    
    disp(['Iteration ', num2str(i), ' - Latitude: ', num2str(lat * 180/pi), ...
          ', Height: ', num2str(h), ', Delta: ', num2str(abs(lat - lat_prev) * 180/pi)]);
    
    if abs(lat - lat_prev) < 1e-12
        disp(['Convergence achieved after ', num2str(i), ' iterations']);
        break;
    end
end

% Convert to degrees
latitude = lat * 180 / pi;
longitude = longitude * 180 / pi;
height = h;

disp(['Final output - Latitude: ', num2str(latitude), ', Longitude: ', num2str(longitude), ', Height: ', num2str(height)]);
end

% Helper function to find UTM zone
function utmZone = findUtmZone(latitude, longitude)
% This is a simplified version of findUtmZone.
% For a complete implementation, see the original code

disp('findUtmZone: Determining UTM zone');
disp(['Input - Latitude: ', num2str(latitude), ', Longitude: ', num2str(longitude)]);

% Check for invalid input
if isnan(latitude) || isnan(longitude) || isinf(latitude) || isinf(longitude)
    disp('ERROR: Invalid coordinates (NaN or Inf)');
    utmZone = '0N';  % Default invalid zone
    return;
end

% Make sure longitude is between -180 and 180
longitude = mod(longitude + 180, 360) - 180;
disp(['Normalized longitude: ', num2str(longitude)]);

% Calculate UTM zone number
zoneNumber = floor((longitude + 180) / 6) + 1;
disp(['Initial zone number: ', num2str(zoneNumber)]);

% Special zones for Norway and Svalbard
if latitude >= 56 && latitude < 64 && longitude >= 3 && longitude < 12
    disp('Special case: Norway region');
    zoneNumber = 32;
end

if latitude >= 72 && latitude < 84
    disp('Special case: Svalbard region');
    if longitude >= 0 && longitude < 9
        zoneNumber = 31;
    elseif longitude >= 9 && longitude < 21
        zoneNumber = 33;
    elseif longitude >= 21 && longitude < 33
        zoneNumber = 35;
    elseif longitude >= 33 && longitude < 42
        zoneNumber = 37;
    end
end

% Determine if in northern or southern hemisphere
if latitude >= 0
    utmZone = [num2str(zoneNumber) 'N'];
    disp('Northern hemisphere');
else
    utmZone = [num2str(zoneNumber) 'S'];
    disp('Southern hemisphere');
end

disp(['Final UTM zone: ', utmZone]);
end

% Helper function to convert from ECEF to UTM
function [E, N, U] = cart2utm(X, Y, Z, utmZone)
% This is a simplified version, assuming cart2utm is already available.
% For a complete implementation, consider using existing functions.

disp('cart2utm: Converting ECEF to UTM coordinates');
disp(['Input ECEF - X: ', num2str(X), ', Y: ', num2str(Y), ', Z: ', num2str(Z)]);
disp(['UTM zone: ', utmZone]);

% Check for invalid input
if isnan(X) || isnan(Y) || isnan(Z) || isinf(X) || isinf(Y) || isinf(Z)
    disp('ERROR: Invalid ECEF coordinates (NaN or Inf)');
    E = NaN;
    N = NaN;
    U = NaN;
    return;
end

% First convert ECEF to geodetic
[lat, lon, h] = cart2geo(X, Y, Z, 5);

% Get the zone number from the UTM zone string
zoneNumber = str2double(utmZone(1:end-1));
disp(['Zone number: ', num2str(zoneNumber)]);

% WGS84 parameters
a = 6378137.0;            % semi-major axis [m]
f = 1/298.257223563;      % flattening

% Convert latitude and longitude to radians
lat_rad = lat * pi / 180;
lon_rad = lon * pi / 180;

% UTM parameters
k0 = 0.9996;              % scale factor
FE = 500000;              % false easting
FN = 0;                   % false northing (default for northern hemisphere)

% Check for southern hemisphere
if utmZone(end) == 'S'
    disp('Southern hemisphere, adding false northing');
    FN = 10000000;        % 10,000,000 meters for southern hemisphere
end

% Central meridian for the zone
lonOrigin = (zoneNumber - 1) * 6 - 180 + 3; % +3 puts origin in middle of zone
lonOrigin_rad = lonOrigin * pi / 180;
disp(['Central meridian: ', num2str(lonOrigin), ' degrees']);

% Calculate UTM parameters
N = a / sqrt(1 - f * (2 - f) * sin(lat_rad)^2);
T = tan(lat_rad)^2;
C = (f / (1 - f)) * cos(lat_rad)^2;
A = (lon_rad - lonOrigin_rad) * cos(lat_rad);

% Calculate M (true distance along the central meridian)
M = a * ((1 - f/4 - 3*f^2/64 - 5*f^3/256) * lat_rad ...
        - (3*f/8 + 3*f^2/32 + 45*f^3/1024) * sin(2*lat_rad) ...
        + (15*f^2/256 + 45*f^3/1024) * sin(4*lat_rad) ...
        - (35*f^3/3072) * sin(6*lat_rad));
disp(['M parameter: ', num2str(M)]);

% Calculate UTM coordinates
E = FE + k0 * N * (A + (1 - T + C) * A^3/6 + (5 - 18*T + T^2 + 72*C - 58) * A^5/120);
N = FN + k0 * (M + N * tan(lat_rad) * (A^2/2 + (5 - T + 9*C + 4*C^2) * A^4/24 ... 
            + (61 - 58*T + T^2 + 600*C - 330) * A^6/720));
U = h; % Height above ellipsoid

disp(['Output UTM - E: ', num2str(E), ', N: ', num2str(N), ', U: ', num2str(U)]);
end