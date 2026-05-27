classdef ISegmentationAlgorithm
    methods (Abstract)
        [segmentedCam, segmentedCameraImg] = SemanticSegmentation(img, imgWidth, imgHeight);
    end
end