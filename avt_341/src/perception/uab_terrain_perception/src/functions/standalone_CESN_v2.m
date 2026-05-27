function ann_modified = standalone_CESN_v2(img_read,Win,Wres,Wout,params_cesn)
%  STANDALONE_CESN_V2 CESN function
  
    inSz = params_cesn.inSz; 
    nTS = params_cesn.nTS; 
    classes = params_cesn.classes;
    imgSize = params_cesn.imgSize; 
    row_sub = params_cesn.row_sub; 
    col_sub = params_cesn.col_sub;
    nRes = params_cesn.nRes; 
    bias = params_cesn.bias; 
    LR = params_cesn.LR; 
    greenBoost = params_cesn.greenBoost;
    testOuts = zeros(inSz,nTS,classes);
    rs_output = zeros(inSz,nTS,3,nRes);
    img_data = data_split(img_read,row_sub,col_sub,nRes);
    % Run ESN
    for pt = 1:nRes
        img_in = img_data{pt,1};
        img_rs = reshape(img_in, [imgSize, 3]);
        input_res = (double(img_rs(:,1))*1000000 + ...
                     double(img_rs(:,2))*1000 + ...
                     double(img_rs(:,3)))/255255255;
        rs_in_res = reshape(input_res,[inSz,nTS]);
        thisInput = vertcat(bias,rs_in_res); % [128+1x32]
        stateHarvest = reservoir_update_cesn(Wres{pt,1}, Win{pt,1}, LR, thisInput, nTS);
        x_te = vertcat(bias, stateHarvest);
        for c = 1:classes
            testOuts(:,:,c) = Wout{pt,c} * x_te;
        end
    
        rec_img = reshape(testOuts,[inSz*nTS,classes]); % [128x32x32] -> [131072x4]
        rec_img(:,1) = rec_img(:,1) + greenBoost;
        [~, catIdx] = max(rec_img, [], 2);
    
        output = zeros(imgSize,3);
        for ohd = 1:size(catIdx,1)
            val = catIdx(ohd,1);
            switch val
                case 1 % High Traf. (green)
                    output(ohd,:) = [000 255 000];
                case 2 % Mod. Traf. (yellow)
                    output(ohd,:) = [255 255 000];
                case 3 % sky (cyan)
                    output(ohd,:) = [000 255 255];
                case 4 % obstruction (red)
                    output(ohd,:) = [255 000 000];
            end
        end
        rs_output(:,:,:,pt) = reshape(output, [inSz, nTS, 3]);
    end
    rs_output = cast(rs_output,'uint8');
    if nRes > 1
        ti_out = imtile(rs_output,'GridSize',[row_sub col_sub]);
    else
        ti_out = rs_output;
    end
    % Smooth the green class:
    green_rgb = [0, 255, 0];
    green_mask = ti_out(:,:,1) == green_rgb(1) & ...
                 ti_out(:,:,2) == green_rgb(2) & ...
                 ti_out(:,:,3) == green_rgb(3);
    se = strel('disk',4);
    BW2 = imdilate(green_mask,se);
    ann_modified = ti_out;
    [row_idx, col_idx] = find(BW2);
    for id = 1:length(row_idx)
        ann_modified(row_idx(id), col_idx(id), :) = green_rgb;
    end
end