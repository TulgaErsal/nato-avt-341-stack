% Define the struct for intrinsic parameters
intrinsics_struct = struct();
intrinsics_struct.FocalLength = [1059.2615537582101, 1060.46731890928]; 
intrinsics_struct.PrincipalPoint = [1027.2509160007201, 811.43962606789705];
intrinsics_struct.ImageSize = [1536, 2048];
intrinsics_struct.RadialDistortion = [-1.3639262962165799E-2, 3.2381354802693399E-2];
intrinsics_struct.TangentialDistortion = [0, 0];
intrinsics_struct.Skew = 0;
intrinsics_struct.K = [1059.2615537582101, 0, 1027.2509160007201;
                0, 1060.46731890928, 811.43962606789705;
                0, 0, 1];

load("resultsFFI.mat")

extrinsics = struct();
extrinsics.T_cam_lidar = tform.A;

% Create YAML string
% Display YAML string

yamlStr = writeCalibrationToYamlFile(intrinsics_struct, extrinsics)




