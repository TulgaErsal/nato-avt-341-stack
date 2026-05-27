function imgParts = data_split(img,row_sub,col_sub,nRes)
%% DATA_SPLIT Image Split into Number of Parallel Reservoirs
    [rows, cols, ~] = size(img);
    partsRows = floor(rows / row_sub); 
    partsCols = floor(cols / col_sub);
    imgParts = cell(nRes,1);
    index = 1;
    for r = 1:partsRows:rows
        for c = 1:partsCols:cols
            rowStart = r;
            rowEnd = r + partsRows - 1;
            colStart = c;
            colEnd = c + partsCols - 1;
            
            imgParts{index} = img(rowStart:rowEnd,colStart:colEnd,:);
            index = index + 1;
        end
    end
end