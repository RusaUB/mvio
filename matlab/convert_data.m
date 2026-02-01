function convert_data(dataDir) 
if nargin < 1
    error('Please provide the data directory path.');
end

% Resolve path if running from matlab/ directory
if ~exist(dataDir, 'dir')
    if exist(fullfile('..', dataDir), 'dir')
        dataDir = fullfile('..', dataDir);
    else
        error('Data directory not found: %s', dataDir);
    end
end

% Get absolute path for clarity
if ismac || isunix
    [~, info] = fileattrib(dataDir);
    absPath = info.Name;
else
    absPath = dataDir; % fileattrib might behave differently on windows sometimes
end
fprintf('Processing data in: %s\n', absPath);

% Find .mat file
matFiles = dir(fullfile(dataDir, '*.mat'));
if isempty(matFiles)
    error('No .mat file found in %s', dataDir);
end
matPath = fullfile(matFiles(1).folder, matFiles(1).name);
fprintf('Loading %s...\n', matPath);

% Load Data
try
    data = load(matPath);
    % Check for generic variables
    if isfield(data, 'Acceleration') && isfield(data, 'AngularVelocity')
        acc = data.Acceleration;
        gyro = data.AngularVelocity;
    elseif isfield(data, 'Position') % Sometimes weird names
        % Try to find by type or guess
        names = fieldnames(data);
        for i=1:length(names)
             val = data.(names{i});
             if isa(val, 'timetable')
                 if any(strcmp(val.Properties.VariableNames, 'X'))
                    % Could be acc or gyro
                    if mean(abs(val.X)) > 2 % Rough gravity check?
                        acc = val;
                    else
                        gyro = val;
                    end
                 end
             end
        end
    else
        error('Could not identify Acceleration and AngularVelocity in .mat file');
    end
catch ME
    error('Failed to load data: %s', ME.message);
end

% Ensure Timetables
if ~isa(acc, 'timetable') || ~isa(gyro, 'timetable')
    warning('Data is not timetable. Assuming structs with Timestamp, X, Y, Z');
    % Handle struct case if needed or error
end

% Synchronize
% Resample gyro to acc timestamps
new_gyro = retime(gyro, acc.Timestamp, 'linear');

% Extract Arrays
t = seconds(acc.Timestamp - acc.Timestamp(1)); % Relative time in seconds
ax = acc.X; ay = acc.Y; az = acc.Z;
gx = new_gyro.X; gy = new_gyro.Y; gz = new_gyro.Z;

% Write CSV
outputCsv = fullfile(dataDir, 'imu.csv');
T = table(t, gx, gy, gz, ax, ay, az);
writetable(T, outputCsv, 'WriteVariableNames', false); % Matches our parser expect
fprintf('Wrote IMU data to %s\n', outputCsv);

% Process Video
videoFiles = dir(fullfile(dataDir, '*.MOV'));
if isempty(videoFiles)
    videoFiles = dir(fullfile(dataDir, '*.mov'));
end

if ~isempty(videoFiles)
    vidPath = fullfile(videoFiles(1).folder, videoFiles(1).name);
    fprintf('Processing video %s...\n', vidPath);
    
    framesDir = fullfile(dataDir, 'frames');
    if ~exist(framesDir, 'dir')
        mkdir(framesDir);
    end
    
    v = VideoReader(vidPath);
    idx = 0;
    while hasFrame(v)
        frame = readFrame(v);
        
        % Timestamp estimation
        % If we assume video starts at t=0 relative to IMU?
        % Or use v.CurrentTime
        t_frame = v.CurrentTime; 
        
        fname = sprintf('%.6f.png', t_frame);
        imwrite(frame, fullfile(framesDir, fname));
        
        if mod(idx, 30) == 0
            fprintf('Saved frame %.3f\n', t_frame);
        end
        idx = idx + 1;
    end
    fprintf('Video processing complete.\n');
else
    warning('No video file found.');
end

end
