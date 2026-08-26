function convert_data(dataDir)
%CONVERT_DATA Convert a MATLAB Mobile IMU + video recording to CSV and PNG frames.
%   CONVERT_DATA(DATADIR) reads the .mat sensor log and the .mov video found
%   in DATADIR, resamples the gyroscope onto the accelerometer timebase, and
%   writes:
%       <dataDir>/imu.csv       columns: t gx gy gz ax ay az (no header row)
%       <dataDir>/frames/*.png  one file per frame, named <timestamp>.png
%
%   All timestamps are in seconds. IMU time is relative to the first
%   accelerometer sample; frame time is relative to the start of the video.
%   The two clocks are NOT aligned with each other -- see extractFrames.

    if nargin < 1
        error('convert_data:NoInput', 'Please provide the data directory path.');
    end

    dataDir = resolveDataDir(dataDir);
    fprintf('Processing data in: %s\n', dataDir);

    [acc, gyro] = loadImu(dataDir);
    writeImuCsv(dataDir, acc, gyro);
    extractFrames(dataDir);
end


function absPath = resolveDataDir(dataDir)
%RESOLVEDATADIR Locate the data directory and return its absolute path.
    if ~exist(dataDir, 'dir')
        candidate = fullfile('..', dataDir);
        if exist(candidate, 'dir')
            dataDir = candidate;   % running from the matlab/ subdirectory
        else
            error('convert_data:DirNotFound', ...
                'Data directory not found: %s', dataDir);
        end
    end

    % fileattrib resolves to an absolute path on every platform, so there is
    % no need to special-case Windows here.
    [ok, info] = fileattrib(dataDir);
    if ok
        absPath = info.Name;
    else
        absPath = dataDir;
    end
end


function [acc, gyro] = loadImu(dataDir)
%LOADIMU Load the accelerometer and gyroscope timetables from the .mat file.
    matFiles = dir(fullfile(dataDir, '*.mat'));
    if isempty(matFiles)
        error('convert_data:NoMatFile', 'No .mat file found in %s', dataDir);
    end
    if numel(matFiles) > 1
        warning('convert_data:MultipleMatFiles', ...
            '%d .mat files found; using %s', numel(matFiles), matFiles(1).name);
    end

    matPath = fullfile(matFiles(1).folder, matFiles(1).name);
    fprintf('Loading %s...\n', matPath);

    try
        data = load(matPath);
    catch ME
        error('convert_data:LoadFailed', ...
            'Failed to load %s: %s', matPath, ME.message);
    end

    if isfield(data, 'Acceleration') && isfield(data, 'AngularVelocity')
        acc  = data.Acceleration;
        gyro = data.AngularVelocity;
    else
        [acc, gyro] = guessImuVariables(data);
    end

    acc  = requireTimetable(acc,  'accelerometer');
    gyro = requireTimetable(gyro, 'gyroscope');
end


function [acc, gyro] = guessImuVariables(data)
%GUESSIMUVARIABLES Identify the sensors by signal magnitude when names differ.
%   A resting accelerometer reads about 9.81 m/s^2 in total magnitude, while a
%   gyroscope reads close to zero. Testing the norm of X/Y/Z is much more
%   robust than testing a single axis: depending on how the device was held,
%   any one accelerometer axis can sit near zero.
    acc  = [];
    gyro = [];

    names = fieldnames(data);
    for i = 1:numel(names)
        val = data.(names{i});
        if ~isa(val, 'timetable')
            continue;
        end
        if ~all(ismember({'X', 'Y', 'Z'}, val.Properties.VariableNames))
            continue;
        end

        magnitude = median(vecnorm([val.X, val.Y, val.Z], 2, 2), 'omitnan');
        if magnitude > 5
            acc = val;      % m/s^2, dominated by gravity
        else
            gyro = val;     % rad/s, near zero unless rotating hard
        end
    end

    if isempty(acc) || isempty(gyro)
        error('convert_data:UnidentifiedSensors', ...
            ['Could not identify Acceleration and AngularVelocity in the .mat ' ...
             'file. Expected timetables with X, Y and Z variables.']);
    end
end


function tt = requireTimetable(tt, label)
%REQUIRETIMETABLE Fail loudly rather than crashing later on a missing method.
    if ~isa(tt, 'timetable')
        error('convert_data:NotATimetable', ...
            ['The %s data is a %s, not a timetable. Convert it to a timetable ' ...
             'before running this script.'], label, class(tt));
    end
end


function writeImuCsv(dataDir, acc, gyro)
%WRITEIMUCSV Resample the gyroscope onto the accelerometer clock and export.

    % Read the row times through the timetable API instead of hard-coding a
    % variable name: MATLAB Mobile calls this column Timestamp, but other
    % exporters call it Time, and acc.Timestamp then errors out.
    accTime = acc.Properties.RowTimes;

    % 'linear' interpolates but does not extrapolate, so accelerometer samples
    % that fall outside the gyroscope's time span come back as NaN.
    gyroSync = retime(gyro, accTime, 'linear');

    valid = ~any(ismissing(gyroSync(:, {'X', 'Y', 'Z'})), 2);
    if ~all(valid)
        warning('convert_data:TrimmedSamples', ...
            'Dropping %d accelerometer samples outside the gyroscope time range.', ...
            sum(~valid));
        accTime  = accTime(valid);
        acc      = acc(valid, :);
        gyroSync = gyroSync(valid, :);
    end

    if isempty(accTime)
        error('convert_data:NoOverlap', ...
            'Accelerometer and gyroscope recordings do not overlap in time.');
    end

    t = seconds(accTime - accTime(1));
    T = table(t, gyroSync.X, gyroSync.Y, gyroSync.Z, acc.X, acc.Y, acc.Z);

    outputCsv = fullfile(dataDir, 'imu.csv');
    writetable(T, outputCsv, 'WriteVariableNames', false);  % parser expects no header
    fprintf('Wrote %d IMU samples to %s\n', height(T), outputCsv);
end


function extractFrames(dataDir)
%EXTRACTFRAMES Save every video frame as <timestamp>.png under frames/.
%   NOTE: frame timestamps are relative to the start of the video, which is
%   not the same instant as the first IMU sample. Estimating that offset is
%   left to the calibration stage downstream.

    % On case-insensitive filesystems both patterns match the same files.
    videoFiles = [dir(fullfile(dataDir, '*.MOV')); dir(fullfile(dataDir, '*.mov'))];
    if ~isempty(videoFiles)
        [~, keep] = unique({videoFiles.name}, 'stable');
        videoFiles = videoFiles(keep);
    end

    if isempty(videoFiles)
        warning('convert_data:NoVideo', 'No video file found in %s', dataDir);
        return;
    end
    if numel(videoFiles) > 1
        warning('convert_data:MultipleVideos', ...
            '%d video files found; using %s', numel(videoFiles), videoFiles(1).name);
    end

    vidPath = fullfile(videoFiles(1).folder, videoFiles(1).name);
    fprintf('Processing video %s...\n', vidPath);

    framesDir = fullfile(dataDir, 'frames');
    if ~exist(framesDir, 'dir')
        mkdir(framesDir);
    end

    v = VideoReader(vidPath);
    idx = 0;
    while hasFrame(v)
        % Read CurrentTime BEFORE readFrame. readFrame advances CurrentTime to
        % the next frame, so sampling it afterwards labels every frame with the
        % timestamp of its successor -- a systematic one-frame offset.
        tFrame = v.CurrentTime;
        frame  = readFrame(v);

        imwrite(frame, fullfile(framesDir, sprintf('%.6f.png', tFrame)));

        if mod(idx, 30) == 0
            fprintf('Saved frame %d at t = %.3f s\n', idx, tFrame);
        end
        idx = idx + 1;
    end

    fprintf('Video processing complete: %d frames written to %s\n', idx, framesDir);
end
