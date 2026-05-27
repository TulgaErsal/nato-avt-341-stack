% ESN Global Parameters
function params_cesn = globalParams
init_img = rand(300, 480);
params_cesn = struct();

params_cesn.Nr = 80; % size of each reservoir
params_cesn.classes = 4;
params_cesn.LR = 1; % smoothing rate; 1 means no smoothing
params_cesn.greenBoost = 0.1; % force green class to show more

% nRes = row_sub * col_sub
params_cesn.row_sub = 1; % how many tiles along the image row to split into?
params_cesn.col_sub = 1; % how many tiles along the image column to split into?
params_cesn.nRes = params_cesn.row_sub*params_cesn.col_sub;
% Color Mapping:

params_cesn.color_codes = [000255000;  % High Traf.
               255255000;  % Med Traf.
               000255255;  % Sky
               255000000]; % Obstructions

params_cesn.cmap = [0, 1, 0;  % green
        1, 1, 0;  % yellow
        0, 1, 1;  % cyan
        1, 0, 0]; % red
% Variable Initialization:

% image size = nTS*inSz
params_cesn.inSz = size(init_img,1)/params_cesn.row_sub; % Nr of updates for each input
params_cesn.nTS = size(init_img,2)/params_cesn.col_sub; % batch size; nTS*inSz=input size to each reservoir; in_res*par=total pcl size

params_cesn.iSz = size(init_img,1)*size(init_img,2);
params_cesn.imgSize = size(init_img,1)*size(init_img,2)/params_cesn.nRes;
params_cesn.Nu_res = params_cesn.imgSize; % [4096x1]
params_cesn.Nu = params_cesn.Nu_res/params_cesn.nTS; % Nu  [128x1x32]
params_cesn.in_res = params_cesn.inSz*params_cesn.nTS;
params_cesn.bias = ones(1,params_cesn.nTS);
end