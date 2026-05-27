classdef KNN < ILidarProcessingAlgorithm
    properties
        KNN_weighted
        pcPostProcessingEnabled
    end
    methods
        function obj = KNN(pcPostProcessingEnabled)
            fprintf("loading KNN parameters (KNN_weighted.mat)\n")
            tmp = load('KNN_weighted.mat');
            obj.KNN_weighted = tmp.KNN_weighted;

            obj.pcPostProcessingEnabled = pcPostProcessingEnabled;
        end

        function segmentedPC = ProcessPointCloud(obj, segmentedPC)
            % KNN to replace LESN
            searchRadius = 10;
            % remove NaN
            nonZeroRows = any(~isnan(segmentedPC.Location), 2);
            filtLoc = segmentedPC.Location(nonZeroRows, :);
        
            density = zeros(size(filtLoc,1),1);
            verticalRange = zeros(size(filtLoc,1),1);
            elevationDifference = zeros(size(filtLoc,1),1);
        
            % Extract features
            denden = 4/3 * pi * searchRadius^3;
            for p = 1:size(filtLoc,1)
                distances = sqrt(sum((filtLoc - filtLoc(p,:)).^2, 2));
                withinRadiusIdx = find(distances < searchRadius);
                neighbors = filtLoc(withinRadiusIdx, :);
                density(p,1) = numel(withinRadiusIdx) / denden;
                verticalRange(p,1) = max(neighbors(:,3)) - min(neighbors(:,3));
                elevationDifference(p,1) = filtLoc(p,3) - mean(neighbors(:,3));
            end
        
            mla_input = [filtLoc, density, verticalRange, elevationDifference];
        
            % Run MLA
            yfit = obj.KNN_weighted.predictFcn(mla_input);
        
            % convert yfit to RGB
            rgbMatrix = zeros(size(filtLoc,1), 3);
            rgbMatrix(yfit == 1, :) = repmat([1, 0, 0], sum(yfit == 1), 1);
            rgbMatrix(yfit == 0, :) = repmat([0, 1, 0], sum(yfit == 0), 1);
        
            % place RGB matrix back into original pcl
            mla_out = zeros(segmentedPC.Count, 3);
            mla_out(nonZeroRows,:) = rgbMatrix;

            knn_pcl = pointCloud(segmentedPC.Location,"Color",mla_out);

            if (obj.pcPostProcessingEnabled)
                segmentedPC = obj.SegmentationPostProcessing(segmentedPC, knn_pcl);
            end
        end
    end
end