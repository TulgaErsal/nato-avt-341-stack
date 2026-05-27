function cmap = dataColorMap()
% Define the colormap used by RUGD dataset.

cmap = [
    000 255 255 % sky
    000 255 000 % trail
    255 255 000 % grass
    255 000 000 % obstruction
    ];

% Normalize between [0 1].
cmap = cmap ./ 255;
end