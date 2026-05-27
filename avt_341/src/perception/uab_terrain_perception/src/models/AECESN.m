classdef AECESN < ISegmentationAlgorithm
    properties
        autoencoderNet
        Win_cesn
        Wres_cesn
        Wout_cesn
        params_cesn
    end
    methods
        function obj = AECESN(modelParams)
            fprintf("loading AECESN parameters (res_weights_singleRes.mat)\n")
            tmp = load('res_weights_singleRes.mat');
            obj.Win_cesn = tmp.Win_cesn;
            obj.Wout_cesn = tmp.Wout_cesn;
            obj.Wres_cesn = tmp.Wres_cesn;

            modelLoadingMsg = sprintf("loading AECESN parameters (%s)\n", modelParams);
            fprintf(modelLoadingMsg);
            tmp = load(modelParams);

            obj.autoencoderNet = tmp.autoencoderNet;
            obj.params_cesn = globalParams();
        end

        function [segmentedImg, resizedSegmentedImg] = SemanticSegmentation(obj, img, imgWidth, imgHeight)
            in_img = predict(obj.autoencoderNet, img);
            in_img = uint8(in_img);
        
            resh_out = reshape(in_img,[300*480,3]);
            for p = 1:300*480
                % Trail
                if resh_out(p,1) < 130 && resh_out(p,2) > 90 && resh_out(p,3) < 110
                    resh_out(p,:) = [0 255 0];
                end
        
                % Grass
                if resh_out(p,1) > 110 && resh_out(p,2) > 120 && resh_out(p,3) < 120
                    resh_out(p,:) = [255 255 0];
                end
        
                % Sky
                if resh_out(p,1) < 150 && resh_out(p,2) > 110 && resh_out(p,3) > 100
                    resh_out(p,:) = [0 255 255];
                end
        
                % Obstruction
                if resh_out(p,1) > 100 && resh_out(p,2) < 140 && resh_out(p,3) < 140
                    resh_out(p,:) = [255 0 0];
                end
            end
            resh_color = reshape(resh_out,[300,480,3]);
        
            segmentedImg = standalone_CESN_v2(in_img,obj.Win_cesn,obj.Wres_cesn,obj.Wout_cesn,obj.params_cesn); % Where is params_cesn defined?
        
            % create mask of the obstuctions
            redChannel = resh_color(:,:,1);
            mask = redChannel == 255 & resh_color(:,:,2) == 0 & resh_color(:,:,3) == 0;
        
            % remove obstruction noise by filtering the mask
            sizeThreshold = 500;
            cleanedMask = bwareaopen(mask, sizeThreshold);
        
            segmentedImg(repmat(cleanedMask, [1, 1, 3])) = resh_color(repmat(cleanedMask, [1, 1, 3]));
        
            resizedSegmentedImg = imresize(segmentedImg,[imgHeight, imgWidth],'nearest');
        end
    end
end