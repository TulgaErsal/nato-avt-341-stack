function [segmentedPC,segmentedImg] = perception(img, lidar, cameraInfo, lidarToCameraTform, segmentationModel, lidarProcessingModel, correctColor)
% downsampledImg = imresize(img,[300,480]);
downsampledImg = imresize(img,[544,688]);
if correctColor
    downsampledImg = imadd(downsampledImg, 75);
end

% Semantic Segmentation
[segmentedImg, resizedSegmentedImg] = segmentationModel.SemanticSegmentation(downsampledImg, cameraInfo.width, cameraInfo.height);

% build cameraIntrinsics struct from CameraInfo message for sensor fusion
imageSize = [cameraInfo.height cameraInfo.width];
focalLength = [cameraInfo.k(1) cameraInfo.k(5)];
principalPoint = [cameraInfo.k(3) cameraInfo.k(6)];
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
cameraToLidarTform= invert(lidarToCameraTform);

% CAM/LIDAR Fusion
segmentedPC = fuseCameraToLidar(resizedSegmentedImg, lidar, camIntrinsics, lidarToCameraTform);

% Process LIDAR
% segmentedPC = lidarProcessingModel.ProcessPointCloud(segmentedPC);

end