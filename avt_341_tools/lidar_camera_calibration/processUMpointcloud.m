function ptCloudOut = processUMpointcloud(ptCloudIn)
% Define the limits for cropping
roi = [-5, 0, -3, 3, -2, 2];
indices = findPointsInROI(ptCloudIn, roi);
ptCloudOut = select(ptCloudIn,indices);
end