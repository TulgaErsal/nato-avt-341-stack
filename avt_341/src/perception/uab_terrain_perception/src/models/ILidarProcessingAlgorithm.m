classdef ILidarProcessingAlgorithm
    methods (Abstract)
        segmentedPC = ProcessPointCloud(segmentedPC);
    end
    methods (Static)
        function segmentedPC = SegmentationPostProcessing(segmentedPC, pcOut)
            % Compare and fix the DL segmentation:
            sum_pco = uint32(segmentedPC.Color(:,1))*1000000 + ...
                uint32(segmentedPC.Color(:,2))*1000 + ...
                uint32(segmentedPC.Color(:,3));
            sum_pcs = uint32(pcOut.Color(:,1))*1000000 + ...
                uint32(pcOut.Color(:,2))*1000 + ...
                uint32(pcOut.Color(:,3));
            for com = 1:segmentedPC.Count
                if sum_pcs(com,1)==255000 && ...
                        sum_pco(com,1)==255000000
                    segmentedPC.Color(com,:) = [000 255 000];
                end
            end
        end
    end
end