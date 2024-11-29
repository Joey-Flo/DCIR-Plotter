% Adjustable Parameters
clear s;

% Serial Communication Parameters
arduinoPort = "COM5";
baudRate = 19200;

% Initialize Serial Port
s = serialport(arduinoPort, baudRate);
s.Timeout = 60000;
pause(2);  % Allow time for the connection to establish

% Adjustable Parameters
DISCHARGECURRENT = 0.9;  % Discharge current in Amps
SOC_VALUE = 95;          % Desired SOC percentage for estimation

% File Parameters
fileDirectory = 'C:\Users\joeyf\OneDrive\Documents\MATLAB\DCIR Data';
fileName = '1000maH_Lithium_Cell_SOC_DCIR(3).txt';  % Data file
filePath = fullfile(fileDirectory, fileName);

% Ensure Directory Exists
if ~isfolder(fileDirectory)
    mkdir(fileDirectory);
end

% Validate File Existence
if ~isfile(filePath)
    error('The file does not exist: %s', filePath);
end

% Open File for Reading
fileID = fopen(filePath, 'r');
if fileID == -1
    error('Failed to open the file: %s', filePath);
end

% Initialize Variables
loop = true;
previousdata = "";
x = zeros(1, 500);  % Time data
y = zeros(1, 500);  % Voltage data
counter = 0;        % Data counter
SampleSize = 500;

% Main Loop to Read File Data
try
    while loop
        data = fgetl(fileID);  % Read line from file
        if data == -1  % End of file
            break;
        end

        if ~isempty(data)
            parsedData = str2double(split(strtrim(data), ';'));

            % Validate and Process Data
            if numel(parsedData) >= 3 && ~strcmp(data, previousdata) && all(~isnan(parsedData))
                counter = counter + 1;
                x(counter) = parsedData(1);  % Time in minutes
                y(counter) = parsedData(2);  % Voltage

                % Exit Conditions
                if counter == SampleSize || parsedData(1) < 0
                    break;
                end

                previousdata = data;  % Update previous data
            end
        end
    end
catch ME
    disp(['Error reading data: ', ME.message]);
end
fclose(fileID);  % Close the file

% Validate Data for Processing
if counter < 2
    error('Insufficient data collected for processing.');
end

% Calculate Battery Capacity
BatteryCapacity = ((counter - 1) / 60) * DISCHARGECURRENT;

% Calculate State of Charge (SOC)
for i = 1:(counter - 1)
    x(i) = 100 - ((((x(i) / 60) * DISCHARGECURRENT) / BatteryCapacity) * 100);
end
x(counter:end) = 0;  % Zero-fill remaining data

% Interpolate Voltage for Desired SOC
try
    y_estimated = interp1(x(1:(counter - 1)), y(1:(counter - 1)), SOC_VALUE, 'linear');
    fprintf('\n\nVoltage: %.4f V, SOC: %.2f %%\n\n', y_estimated, SOC_VALUE);

    % Send Data to Arduino
    result = sprintf('3;%.4f;%.2f', y_estimated, DISCHARGECURRENT);
    writeline(s, result);
    disp(['Discharging Battery to ', num2str(SOC_VALUE), '% SOC']);
catch ME
    disp(['Error during interpolation or communication: ', ME.message]);
end

% Monitor and Display Serial Data
loop = true;
try
    while loop
        data = readline(s);  % Read serial data
        if ~isempty(data)
            parsedData = str2double(split(strtrim(data), ';'));
            if numel(parsedData) >= 3 && all(~isnan(parsedData))

                % Example Exit Condition
                if parsedData(1) == 1
                    loop = false;
                end
            end
        end
    end
catch ME
    disp(['Error reading serial data: ', ME.message]);
end

% Final Message
disp(['Battery Successfully Discharged to ', num2str(SOC_VALUE), '% SOC']);
clear s;
