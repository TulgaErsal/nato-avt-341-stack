function yamlStr = writeCalibrationToYamlFile(intrinsics, extrinsics)
    yamlStr = createYAMLstring(intrinsics, extrinsics);

    % Get the current date and time
    currentTime = datetime('now', 'Format', 'yyyy-MM-dd-HH-mm-ss');
    
    % Create new file name
    newFileName = sprintf('%s-camchain-matlab.yaml', currentTime);
    
    % Write YAML string to file
    fid = fopen(newFileName, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', newFileName);
    end
    fprintf(fid, '%s', yamlStr);
    fclose(fid);

    fprintf('YAML output has been written to %s\n', newFileName);
end

function yamlStr = createYAMLstring(intrinsics, extrinsics)
    % Create YAML-formatted text string from intrinsics and extrinsics structs

    % Intrinsics
    intrinsics_combined = sprintf('[%.15g, %.15g, %.15g, %.15g]', ...
        intrinsics.FocalLength(1), intrinsics.FocalLength(2), ...
        intrinsics.PrincipalPoint(1), intrinsics.PrincipalPoint(2));
    
    % Distortion Coefficients
    distortion_coeffs = [intrinsics.RadialDistortion, intrinsics.TangentialDistortion];

    distortion_coeffs_str = sprintf('[%.15g, %.15g, %.15g, %.15g]', ...
        distortion_coeffs(1), distortion_coeffs(2), ...
        distortion_coeffs(3), distortion_coeffs(4));

    % Image Size (Resolution)
    resolution = sprintf('[%d, %d]', intrinsics.ImageSize(2), intrinsics.ImageSize(1));

    % Transformation Matrix (T_cam_lidar)
    T_cam_lidar = sprintf('- [%.15g, %.15g, %.15g, %.15g]\n  - [%.15g, %.15g, %.15g, %.15g]\n  - [%.15g, %.15g, %.15g, %.15g]\n  - [%.15g, %.15g, %.15g, %.15g]', ...
        extrinsics.T_cam_lidar(1,1), extrinsics.T_cam_lidar(1,2), extrinsics.T_cam_lidar(1,3), extrinsics.T_cam_lidar(1,4), ...
        extrinsics.T_cam_lidar(2,1), extrinsics.T_cam_lidar(2,2), extrinsics.T_cam_lidar(2,3), extrinsics.T_cam_lidar(2,4), ...
        extrinsics.T_cam_lidar(3,1), extrinsics.T_cam_lidar(3,2), extrinsics.T_cam_lidar(3,3), extrinsics.T_cam_lidar(3,4), ...
        extrinsics.T_cam_lidar(4,1), extrinsics.T_cam_lidar(4,2), extrinsics.T_cam_lidar(4,3), extrinsics.T_cam_lidar(4,4));

    % Construct YAML string
    yamlStr = sprintf(['cam0:\n' ...
        '  camera_model: pinhole\n' ...
        '  intrinsics: %s\n' ...
        '  distortion_model: plumb_bob\n' ...
        '  distortion_coeffs: %s\n' ...
        '  resolution: %s\n' ...
        '  T_cam_lidar:\n' ...
        '  %s\n'], ...
        intrinsics_combined, distortion_coeffs_str, resolution, T_cam_lidar);
end




