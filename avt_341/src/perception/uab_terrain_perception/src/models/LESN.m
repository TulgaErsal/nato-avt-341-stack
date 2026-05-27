classdef LESN < ILidarProcessingAlgorithm
    properties
        bias
        in_res
        inSz
        LR
        Nr
        nTS
        Nu
        Nu_res
        par
        Win
        Wout
        Wres
        pcPostProcessingEnabled
    end
    methods
        function obj = LESN(pcPostProcessingEnabled)
            fprintf("loading LESN parameters (LESN_32R_Z_4k.mat)\n")
            tmp = load('LESN_32R_Z_4k.mat');
            obj.bias = tmp.bias;
            obj.in_res = tmp.in_res;
            obj.inSz = tmp.inSz;
            obj.LR = tmp.LR;
            obj.Nr = tmp.Nr;
            obj.nTS = tmp.nTS;
            obj.Nu = tmp.Nu;
            obj.Nu_res = tmp.Nu_res;
            obj.par = tmp.par;
            obj.Win = tmp.Win;
            obj.Wout = tmp.Wout;
            obj.Wres = tmp.Wres;

            obj.pcPostProcessingEnabled = pcPostProcessingEnabled;
        end

        function segmentedPC = ProcessPointCloud(obj, segmentedPC)
            % match the size for LESN
            % create empty pcl of correct size
            desiredNumPoints = 29184;
            init_loc = zeros(desiredNumPoints, 3);
            % get segPC locations
            locations = segmentedPC.Location;
            % Find indices of non-NaN & extract
            nonNaNIndices = all(~isnan(locations), 2);
            nonNaNLocations = locations(nonNaNIndices, :);
            % Update lesnPC locations with non-NaN locations from segPC
            init_loc(1:size(nonNaNLocations, 1), :) = nonNaNLocations;
            % Update lesnPC with the new locations
            lesnPC = pointCloud(init_loc);

            z_data = lesnPC.Location;
            for pr = 1:obj.par
                in_pcl = z_data(((pr-1)*obj.in_res+1):(obj.in_res*pr),3); % Z
                rs_in_pcl = reshape(in_pcl,[obj.inSz,obj.nTS]);
                thisInput = vertcat(obj.bias,rs_in_pcl); % [128+1x32]
                stateHarvest_test(:,:,pr) = reservoir_update(obj.Wres, obj.Win, obj.LR, ...
                    thisInput, obj.nTS, pr);
                x_te(:,:,pr) = vertcat(obj.bias, stateHarvest_test(:,:,pr));
                testOuts(:,:,pr) = obj.Wout(:,:,pr) * x_te(:,:,pr);
            end
            rec_pcl = reshape(testOuts,[obj.inSz*obj.nTS*obj.par,1]); % [128x32x32] -> [131072x1]
        
            color_pcl = zeros(obj.inSz*obj.nTS*obj.par,3); % [131072x3]
            for p = 1:lesnPC.Count
                if rec_pcl(p,1) >= 0.8 % this value is kind of arbitrary...
                    rec_pcl(p,1) = 1;
                    color_pcl(p,:) = [255 000 000];
                else
                    rec_pcl(p,1) = 0;
                    color_pcl(p,:) = [000 255 000];
                end
            end
            % LESN segmentation is complete, size 29184
            lesnPC = pointCloud(lesnPC.Location,"Color",color_pcl);
            % Convert LESN seg to full pcl size
            % take non-zero locations and mask color onto full 65536 pcl
            lesnPC_full = pointCloud(segmentedPC.Location);
            nonZeroIndices = any(z_data ~= 0, 2); % Non-zero locations
            nonZeroLocations = z_data(nonZeroIndices, :);
            nonZeroColors = lesnPC.Color(nonZeroIndices, :);
            lesnPC_full.Color = zeros(size(segmentedPC.Location, 1), 3);
            % Match non-zero locations from pc2 to locations in pc1 and apply colors
            for i = 1:size(nonZeroLocations, 1)
                % Find the matching index in pc1 (assuming exact match exists)
                [~, idx] = ismember(nonZeroLocations(i,:), segmentedPC.Location, 'rows');
                % Apply color from lesnPC to matching loc in lesnPC_full
                if idx > 0 % Check if a matching location was found
                    lesnPC_full.Color(idx, :) = nonZeroColors(i, :);
                end
            end

            if (obj.pcPostProcessingEnabled)
                segmentedPC = obj.SegmentationPostProcessing(segmentedPC, lesnPC_full);
            end
        end
    end
end