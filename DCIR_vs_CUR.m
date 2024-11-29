
loop = true;
previousdata = "";
counter = 0;

% Adjustable Parameters
TESTMODE = true; % Set to `true` to log live data; `false` to read from an existing file

CURRENTSTEPSIZE = 0.04; % in Amps
MAXCURRENT = 1; % in Amps
CUTOFFVOLTAGE = 2.65;

% Adjustable Parameters for Text File
fileDirectory = 'C:\Users\joeyf\OneDrive\Documents\MATLAB\DCIR Data';
fileName = 'DCIR_Current2_data.txt';
filePath = fullfile(fileDirectory, fileName);

% Ensure the directory exists
if ~isfolder(fileDirectory)
    mkdir(fileDirectory);
end

% Check and handle file creation
if ~isfile(filePath)
    disp('File does not exist. Creating new file...');
    fileID = fopen(filePath, 'w'); % Create a new file
    if fileID == -1
        error('Failed to create the file: %s', filePath);
    end
    fclose(fileID);
else
    disp(['File already exists: ', filePath]);
end

% Initialize Variables
SAMPLEAMOUNT = MAXCURRENT / CURRENTSTEPSIZE;
CURRENTSTEPSIZEFLOAT = num2str(CURRENTSTEPSIZE);
SAMPLEAMOUNTFLOAT = num2str(SAMPLEAMOUNT);
CUTOFFVOLTAGE = num2str(CUTOFFVOLTAGE);
x = zeros(1, SAMPLEAMOUNT);
y = zeros(1, SAMPLEAMOUNT);
if TESTMODE == true
    clear s;
    arduinoPort = "COM5";
    baudRate = 19200;
    s = serialport(arduinoPort, baudRate);
    s.Timeout = 100;
    pause(2); % Allow time for connection setup
    disp('Reading data from DCIR Plotter...');
    % Prepare for live data logging
    result = strcat('2;', CURRENTSTEPSIZEFLOAT, ';', SAMPLEAMOUNTFLOAT, ';', CUTOFFVOLTAGE);
    writeline(s, result);

    % Open file for writing
    fileID = fopen(filePath, 'a'); % Open file in append mode
    if fileID == -1
        error('Failed to open the file for writing: %s', filePath);
    end
    fprintf('Logging data to %s\n', filePath);
else
    % Open file for reading
    fileID = fopen(filePath, 'r');
    if fileID == -1
        error('Failed to open the file for reading: %s', filePath);
    end
    disp(['Reading lines from the file: ', filePath]);
    fseek(fileID, 0, 'bof'); % Reset file pointer to the beginning
end

% Main Loop
while loop
    try
        if TESTMODE
            data = readline(s); % Read line from serial monitor
            if ~isempty(data)
                fprintf(fileID, '%s\n', data); % Write data to file
            end
        else
            data = fgetl(fileID); % Read line from file
            if data == -1
                loop = false; % End loop at the end of the file
                continue;
            end
        end

        if ~isempty(data)
            parsedData = str2double(split(strtrim(data), ';'));
            if numel(parsedData) >= 2 && ~strcmp(data, previousdata) && all(~isnan(parsedData))
                fprintf('Current: %.2f A, DCIR: %.2f mOhms\n', parsedData(1), parsedData(2));
                counter = counter + 1;
                x(counter) = parsedData(1);
                y(counter) = parsedData(2);

                if parsedData(1) == MAXCURRENT
                    counter = 0;
                    loop = false;
                end
                if parsedData(2) == 1
                    disp('DATA FAILED: CHARGE BATTERY');
                    counter = 0;
                    loop = false;
                end
                previousdata = data; % Update previous data
            end
        end
    catch ME
        % Handle any errors during the reading process
        disp(['Error reading data: ', ME.message]);
    end
end

% Close the file
fclose(fileID);
if TESTMODE
    disp('Data logging completed.');
else
    disp('Finished reading from the file.');
end

% Filter out (0, 0) and (1, 1) points
validIndices = ~((x == 0 & y == 0) | (x == 1 & y == 1));
x = x(validIndices);
y = y(validIndices);

% Plot the data
figure;
plot(x, y, 'o', 'LineWidth', 2);
y_max = max(y);
y_min = min(y);
ylim([(y_min - 5) (y_max + 5)]);
xlabel('Current (A)');
ylabel('DCIR (mOhms)');
title('DCIR vs. Current');
grid on;

% Fit and overlay polynomial
p = polyfit(x, y, 6);
y_fit = polyval(p, x);
hold on;
plot(x, y_fit, '-r', 'LineWidth', 2);
hold off;
