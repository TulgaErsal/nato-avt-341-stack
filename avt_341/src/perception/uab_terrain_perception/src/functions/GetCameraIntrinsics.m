function cameraParams = GetCameraIntrinsics(imgWidth, imgHeight)
    if imgWidth == 960 && imgHeight == 540
        tmp = load('calibration_images/FLIR_parameters.mat');
        cameraParams = tmp.params;
    elseif imgWidth == 2448 && imgHeight == 2048
        tmp = load('calibration_images/FLIR_parameters_highRes.mat');
        cameraParams = tmp.params;
    elseif imgWidth == 1920 && imgHeight == 1208
        tmp = load('calibration_images/left_camera_parameters_MSU.mat');
        cameraParams = tmp.params;
    else
        fprintf("%ux%u is not a supported camera resolution",imgWidth,imgHeight);
    end
end