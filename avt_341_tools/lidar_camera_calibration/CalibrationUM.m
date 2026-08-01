%% Load and Preprocess Bag File
bag = ros2bagreader('..\Bags\UM');
% Select a time window where all four checkerboard corners are visible
bag = select(bag, "Time", [bag.StartTime + 10, bag.StartTime + 120]);
% Extract camera and LIDAR messages from the bag
camBag = select(bag, 'Topic', '/flir_camera/image_raw');
lidarBag = select(bag, 'Topic', '/ouster/points');
camMsgs = readMessages(camBag);
lidarMsgs = readMessages(lidarBag);

%% Synchronize Timestamps Between Camera and LiDAR
% Convert message timestamps to timetables
ttCam = timetable(camBag);
ttLidar = timetable(lidarBag);
% Normalize both time vectors to start from the same reference (t = 0)
tMin = min([ttCam.Time(1), ttLidar.Time(1)]);
tCam = seconds(ttCam.Time - tMin);
tLidar = seconds(ttLidar.Time - tMin);

%% Match Closest Camera Image to Each LIDAR Scan (# Images > # Clouds)
% For each LiDAR scan, find the closest-in-time camera image within 0.1 seconds
k = 1;
idx = zeros(length(tLidar), 2);  % Store matching indices: [camIdx, lidarIdx]
% Process every 10th scan to reduce computation time.
% Adjust this value if the number of matched image–scan pairs is too low or too high.
step = 10;  
for idxLidar = 1:step:length(tLidar)
    [val, idxCam] = min(abs(tLidar(idxLidar) - tCam));
    if val <= 0.1
        idx(k, :) = [idxCam, idxLidar];
        k = k + 1;
    else
        disp(['Scan ', num2str(idxLidar), ' is unmatched.']);
    end
end
% Remove unmatched entries
rowsToRemove = all(idx == 0, 2);
idx(rowsToRemove, :) = [];

%% Prepare Output Directories for Extracted Data
% Set paths for saving extracted images and point clouds
imageFilesPath = fullfile(tempdir, 'AAImages');
pcFilesPath = fullfile(tempdir, 'AAPointClouds');
% Clean or create the directories
if exist(imageFilesPath, 'dir')
    delete(fullfile(imageFilesPath, '*'));  % Delete existing files only
else
    mkdir(imageFilesPath);
end
% Clean or create the point cloud directory
if exist(pcFilesPath, 'dir')
    delete(fullfile(pcFilesPath, '*'));
else
    mkdir(pcFilesPath);
end

%% Extract and Save Matched Image–Point Cloud Pairs
% Loop through matched indices and write data to files
step = 1;
for i = 1:step:length(idx)
    progress = round(100 * i / length(idx), 1);  % Display progress
    disp(['Progress: ', num2str(progress), '%']);
    % Read and convert ROS messages
    I = rosReadImage(camMsgs{idx(i, 1)});
    pc = pointCloud(rosReadXYZ(lidarMsgs{idx(i, 2)}));
    
    % Optional: Define function if point clouds need cleaning
    % Apply preprocessing to point cloud (e.g., cropping to ROI)
    pc = processUMpointcloud(pc);
    
    % Generate zero-padded filenames (e.g., 0001.png, 0001.pcd)
    n_strPadded = sprintf('%04d', i);
    imageFileName = fullfile(imageFilesPath, [n_strPadded, '.png']);
    pcFileName = fullfile(pcFilesPath, [n_strPadded, '.pcd']);
    % Save image and point cloud
    imwrite(I, imageFileName);
    pcwrite(pc, pcFileName);
end

%% Run Camera–LiDAR Calibration Tool
% Define checkerboard and padding parameters (units: mm)
checkerSize = 60;             % Checkerboard square size
padding = [40, 30, 40, 30];   % Padding around the checkerboard

% Launch the interactive calibration GUI
lidarCameraCalibrator(imageFilesPath, pcFilesPath, checkerSize, padding);