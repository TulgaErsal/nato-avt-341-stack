classdef DeepLabV3 < ISegmentationAlgorithm
    properties
        cmap
        classes
        net
    end
    methods
        function obj = DeepLabV3(modelParams)
            modelLoadingMsg = sprintf("loading DLV3 parameters (%s)\n", modelParams);
            fprintf(modelLoadingMsg);
            tmp = load(modelParams);

            obj.classes = tmp.classes;
            obj.cmap = tmp.cmap;
            obj.net = tmp.net;
        end

        function [segmentedSmall, segmentedCameraImg] = SemanticSegmentation(obj, img, imgWidth, imgHeight)
            segmentedCam = semanticseg(img,obj.net,Classes=obj.classes);
            segmentedSmall = labeloverlay(img,segmentedCam,'Colormap',obj.cmap,'Transparency',0);
            segmentedCameraImg = imresize(segmentedSmall,[imgWidth, imgHeight],'nearest');
        end
    end
end