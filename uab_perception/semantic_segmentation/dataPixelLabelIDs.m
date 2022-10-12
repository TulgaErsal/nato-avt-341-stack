function labelIDs = dataPixelLabelIDs()
% Return the label IDs corresponding to each class.
% Any classes to remove are combined with genericObstruction
% Note that the Other/Void class are excluded below.

labelIDs = { ...
    
    % Combine similar classes with square brackets:
    [000 102 000; ... % grass
     108 064 020; ... % dirt 
     101 031 255; ... % uphill
     137 149 009]     % downhill

    [000 255 000] % tree
    [000 153 153] % pole
    [000 000 255] % sky
    [255 255 000] % vehicle
    [102 000 000] % log
    [204 153 255] % person
    [255 153 204] % bush

    [170 170 170; ... % concrete
     064 064 064]     % asphalt

    [041 121 255; ... % barrier
     255 000 127; ... % object
     255 000 000; ... % building
     102 000 204]     % fence

    [134 255 239; ... % puddle
     000 128 255]     % water

    [099 066 034] % mud
    [110 022 138] % rubble

    };
end