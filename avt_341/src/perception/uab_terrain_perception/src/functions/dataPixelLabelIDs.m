function labelIDs = dataPixelLabelIDs()
% Return the label IDs corresponding to each class.
% Any classes to remove are combined with genericObstruction
% Note that the Other/Void class are excluded below.

labelIDs = { ...
    
    [000 255 255] % sky
    [000 255 000] % trail
    [255 255 000] % grass
    [255 000 000] % obstruction

    };
end