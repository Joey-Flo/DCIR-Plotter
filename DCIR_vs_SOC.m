

% Adjustable Parameters
TESTMODE = false; % Testing new data or reading old data
MINVOLTAGE = 2.65; % in Volts
DISCHARGECURRENT = 0.9; % in Amps
INTERVAL = 60000; % Use 60000ms (1min)

% Adjustable Parameters for Text File
fileDirectory = 'C:\Users\joeyf\OneDrive\Documents\MATLAB\DCIR Data';
fileName = '1000maH_Lithium_Cell_SOC_DCIR(2).txt'; % New file name if testing new data
filePath = fullfile(fileDirectory, fileName);

% Ensure the directory exists
if ~isfolder(fileDirectory)
    mkdir(fileDirectory);
end

% Initialize variables
loop = true;
previousdata = "";
disp('Reading data from Discharge Plotter...');
x = zeros(1, 500);
y = zeros(1, 500);
z = zeros(1, 500);
counter = 0;
SampleSize = 500;

% Prepare Serial Communication
MINVOLTAGEstr = num2str(MINVOLTAGE);
DISCHARGECURRENTstr = num2str(DISCHARGECURRENT);
INTERVALstr = num2str(INTERVAL);
result = strcat('1;', MINVOLTAGEstr, ';', DISCHARGECURRENTstr, ';', INTERVALstr);

% Open file for logging or reading
if TESTMODE == true
    clear s;

% Adjustable Parameters
    arduinoPort = "COM5";
    baudRate = 19200;
    s = serialport(arduinoPort, baudRate);
    s.Timeout = 100;
    pause(2);
    % Open the file for writing
    fileID = fopen(filePath, 'w');
    if fileID == -1
        error('Failed to create the file.');
    end
    writeline(s, result);
    fprintf('Logging data to %s\n', filePath);
else
    % Open the file for reading
    if ~isfile(filePath)
        error('The file does not exist: %s', filePath);
    end
    fileID = fopen(filePath, 'r');
    if fileID == -1
        error('Failed to open the file: %s', filePath);
    end
    disp(['Reading lines from the file: ', filePath]);
    fseek(fileID, 0, 'bof'); % Reset file pointer to the beginning
end

% Main Loop
while loop
    try
        if TESTMODE == true
            data = readline(s); % Read line from serial monitor
            fprintf(fileID, '%s\n', data); % Write to file
        else
            data = fgetl(fileID); % Read line from file
            if data == -1
                loop = false; % End loop if end-of-file is reached
                continue;
            end
        end

        if ~isempty(data)
            parsedData = str2double(split(strtrim(data), ';'));

            if numel(parsedData) >= 3 && ~strcmp(data, previousdata) && all(~isnan(parsedData))
                fprintf('Time: %.2f mins, Voltage %.4f v, DCIR: %.2f mOhms\n', ...
                        parsedData(1), parsedData(2), parsedData(3));
                counter = counter + 1;
                x(counter) = parsedData(1);
                y(counter) = parsedData(2);
                z(counter) = parsedData(3);

                if counter == SampleSize
                    loop = false;
                end

                if parsedData(3) <= 0
                    y(counter:end) = 0;
                    z(counter:end) = z(counter - 1);
                    if parsedData(2) == 0 || parsedData(2) < MINVOLTAGE
                        disp("Min Voltage Exceeded");
                    elseif parsedData(2) == 1
                        disp("DCIR exponential increase detected");
                    elseif parsedData(2) == 2
                        disp("Battery can no longer supply current");
                    end
                    loop = false;
                end

                if parsedData(1) < 0
                    loop = false;
                end

                previousdata = data; % Update previous data
            end
        end
    catch ME
        disp(['Error reading data: ', ME.message]);
        break
    end
end

% Close the file
if TESTMODE == true
    fclose(fileID);
    disp('Data logging completed.');
else
    fclose(fileID);
    disp('Finished reading from the file.');
end

% Process Data
xNonZero = find(x ~= 0);
BatteryCapacity = (((counter - 1) / 60) * DISCHARGECURRENT) * 0.96;
fprintf('Battery Capacity: %.2f Ah\n', BatteryCapacity);
fprintf('Min Voltage: %.2f v\n', y(counter - 1));

for i = 1:(counter - 1)
    x(i) = 100 - ((((x(i) / 60) * DISCHARGECURRENT * 0.96) / BatteryCapacity) * 100);
end
x((counter - 1):end) = 0;

% Plot Voltage
figure;
subplot(2, 1, 1);
ax = gca;
ax.XDir = 'reverse';
y_max = max(y);
y_min = min(y);
plot(x(1:(counter - 1)), y(1:(counter - 1)), '-r', 'LineWidth', 2);
xlabel('SOC (%)');
ylabel('Voltage (volts)');
ylim([2.75 4.25]);
xlim([0, 100]);
ax.XDir = 'reverse';
title('Voltage vs. SOC');
grid on;

% Plot DCIR
subplot(2, 1, 2);
ax = gca;
ax.XDir = 'reverse';
if z(counter - 1) > (z(counter - 2) + z(counter - 2) * 0.20)
    SUBTRACTVAL = 2;
else
    SUBTRACTVAL = 1;
end
z_max = max(z(1:counter - SUBTRACTVAL));
z_min = min(z);
coefficients = polyfit(x(1:counter-SUBTRACTVAL), z(1:counter-SUBTRACTVAL), 7); % Fit a linear model (degree 1)
z_fit = polyval(coefficients, x(1:counter-SUBTRACTVAL)); % Calculate fitted values
plot(x(1:counter), z(1:counter), '-b', 'LineWidth', 2);
hold on;
plot(x(1:counter-SUBTRACTVAL), z_fit, '-r', 'LineWidth', 2);
xlabel('SOC (%)');
ylabel('DCIR (mOhms)');
ylim([(z_min - 5) (z_max + 5)]);
xlim([0, 100]);
ax.XDir = 'reverse';
title('DCIR vs. SOC');
grid on;
