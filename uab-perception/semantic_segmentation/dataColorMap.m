function cmap = dataColorMap()
% Define the colormap used by RUGD dataset.

cmap = [
    000 102 000 % grass
    000 255 000 % tree
    000 153 153 % pole
    000 000 255 % sky
    255 255 000 % vehicle
    102 000 000 % log
    204 153 255 % person
    255 153 204 % bush
    170 170 170 % concrete
    041 121 255 % barrier
    134 255 239 % puddle
    099 066 034 % mud
    110 022 138 % rubble
    ];

% Normalize between [0 1].
cmap = cmap ./ 255;
end