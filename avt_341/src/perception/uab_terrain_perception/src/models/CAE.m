classdef CAE < ISegmentationAlgorithm
    properties
        autoencoderNet
        labelIDs
        classNames
    end
    methods
        function obj = CAE(modelParams)
            modelLoadingMsg = sprintf("loading CAE parameters (%s)\n", modelParams);
            fprintf(modelLoadingMsg);
            tmp = load(modelParams);

            obj.autoencoderNet = tmp.net;
            obj.labelIDs = tmp.labelIDs;
            obj.classNames = tmp.classNames;
        end

        function [segmentedImg, resizedSegmentedImg] = SemanticSegmentation(obj, img, imgWidth, imgHeight) 
            resizeImage = imresize(img,[544 688],"bicubic");
            if canUseGPU
                batchGpu = gpuArray(single(resizeImage));
                pxdsResults = semanticseg(batchGpu, obj.autoencoderNet, 'ExecutionEnvironment', 'gpu', Classes=obj.classNames);
            else
                pxdsResults = semanticseg(resizeImage, obj.autoencoderNet, Classes=obj.classNames);
            end
            stringMatrix = string(pxdsResults);
            segmentedImg = zeros(544, 688, 3, 'uint8');

             % Vectorized conversion from string labels to RGB
            [~, idx] = ismember(stringMatrix, obj.classNames);  % Find class indices
            segmentedImg(:, :, 1) = reshape(obj.labelIDs(idx, 1), [544, 688]);  % Red channel
            segmentedImg(:, :, 2) = reshape(obj.labelIDs(idx, 2), [544, 688]);  % Green channel
            segmentedImg(:, :, 3) = reshape(obj.labelIDs(idx, 3), [544, 688]);  % Blue channel

            resizedSegmentedImg = imresize(segmentedImg,[imgHeight imgWidth],"nearest");
        end
    end
end