classdef GridBuilder
    properties
        Resolution {mustBeNumeric}
        Width {mustBeNumeric}
        Height {mustBeNumeric}
        Size {mustBeNumeric}
        HighTraffGrid occupancyMap
        MediumTraffGrid occupancyMap
        LowTraffGrid occupancyMap
        ObstacleGrid occupancyMap
        GridOffsetX {mustBeNumeric}
        GridOffsetY {mustBeNumeric}
        HighConfidenceVal {mustBeNumeric}
        LowConfidenceVal {mustBeNumeric}
        HighTraffCost {mustBeNumeric}
        MediumTraffCost {mustBeNumeric}
        LowTraffCost {mustBeNumeric}
    end
    methods
        function obj = GridBuilder(width, ...
                                    height, ...
                                    res, ...
                                    gridOffsetX, ...
                                    gridOffsetY, ...
                                    defaultTerrainCellVal, ...
                                    defaultObstacleCellVal, ...
                                    highConfidenceVal, ...
                                    lowConfidenceVal, ...
                                    highTraffCost, ...
                                    medTraffCost, ...
                                    lowTraffCost)
            % Grid properties
            obj.Resolution = res;
            obj.Width = width;
            obj.Height = height;
            obj.Size = [obj.Width, obj.Height];
            obj.GridOffsetX = gridOffsetX;
            obj.GridOffsetY = gridOffsetY;

            % Grid objects
            obj.HighTraffGrid = occupancyMap(defaultTerrainCellVal * ones(obj.Size));
            obj.MediumTraffGrid = occupancyMap(defaultTerrainCellVal * ones(obj.Size));
            obj.LowTraffGrid = occupancyMap(defaultTerrainCellVal * ones(obj.Size));
            obj.ObstacleGrid = occupancyMap(defaultObstacleCellVal * ones(obj.Size));

            % Confidence values
            obj.HighConfidenceVal = highConfidenceVal;
            obj.LowConfidenceVal = lowConfidenceVal;

            % Trafficability class costs
            obj.HighTraffCost = highTraffCost;
            obj.MediumTraffCost = medTraffCost;
            obj.LowTraffCost = lowTraffCost;
        end

        function [xs, ys] = UpdateHighTraffCells(obj, trailLocations)
            highTraffConfidence = obj.HighConfidenceVal;   % confidence of high trafficability at these cell locations
            mediumTraffConfidence = obj.LowConfidenceVal;  %       ""      medium      ""     
            lowTraffConfidence = obj.LowConfidenceVal;     %       ""      low         ""
            [xs, ys] = UpdateTerrainCells(obj, trailLocations, highTraffConfidence, mediumTraffConfidence, lowTraffConfidence);
        end

        function [xs, ys] = UpdateMediumTraffCells(obj, grassLocations)
            highTraffConfidence = obj.LowConfidenceVal;    % confidence of high trafficability at these cell locations
            mediumTraffConfidence = obj.HighConfidenceVal; %       ""      medium      ""     
            lowTraffConfidence = obj.LowConfidenceVal;     %       ""      low         ""
            [xs, ys] = UpdateTerrainCells(obj, grassLocations, highTraffConfidence, mediumTraffConfidence, lowTraffConfidence);
        end

        function [xs, ys] = UpdateLowTraffCells(obj, vegLocations)
            highTraffConfidence = obj.LowConfidenceVal;   % confidence of high trafficability at these cell locations
            mediumTraffConfidence = obj.LowConfidenceVal; %       ""      medium      ""     
            lowTraffConfidence = obj.HighConfidenceVal;   %       ""      low         ""
            [xs, ys] = UpdateTerrainCells(obj, vegLocations, highTraffConfidence, mediumTraffConfidence, lowTraffConfidence);
        end

        function [xs, ys] = UpdateTerrainCells(obj, pcLocations, highTraffConfidence, mediumTraffConfidence, lowTraffConfidence)
            if isempty(pcLocations)
                xs = [];
                ys = [];
                return;
            end

            % we got a terrain observation, so that implies there isn't an
            % obstacle at that location
            obstacleConfidence = 0.2; % todo: parameterize?

            [xs, ys] = GetUpdatedCells(obj, pcLocations);
            updateOccupancy(obj.HighTraffGrid, [xs, ys], highTraffConfidence, 'grid');
            updateOccupancy(obj.MediumTraffGrid, [xs, ys], mediumTraffConfidence, 'grid');
            updateOccupancy(obj.LowTraffGrid, [xs, ys], lowTraffConfidence, 'grid');
            updateOccupancy(obj.ObstacleGrid, [xs, ys], obstacleConfidence, 'grid');
        end

        function [xs, ys] = UpdateObstructionCells(obj, pcLocations)
            if isempty(pcLocations)
                xs = [];
                ys = [];
                return;
            end

            [xs, ys] = GetUpdatedCells(obj, pcLocations);
            obstacleConfidence = 0.8; % todo: parameterize?
            updateOccupancy(obj.ObstacleGrid, [xs,ys], obstacleConfidence,'grid');
        end

        function [xs, ys] = GetUpdatedCells(obj, croppedArea)
            xs = rmmissing(ceil((croppedArea(:,1) - obj.GridOffsetX) / obj.Resolution));
            ys = rmmissing(ceil((croppedArea(:,2) - obj.GridOffsetY) / obj.Resolution));
        end

        function [grid, modifiedCellIdxs] = GetCroppedLowTraffGrid(obj, croppedArea)
            [grid, modifiedCellIdxs] = GetRosGrid(obj, obj.LowTraffGrid, croppedArea);
        end

        function [grid, modifiedCellIdxs] = GetCroppedMediumTraffGrid(obj, croppedArea)
            [grid, modifiedCellIdxs] = GetRosGrid(obj, obj.MediumTraffGrid, croppedArea);
        end

        function [grid, modifiedCellIdxs] = GetCroppedHighTraffGrid(obj, croppedArea)
            [grid, modifiedCellIdxs] = GetRosGrid(obj, obj.HighTraffGrid, croppedArea);
        end

        function [grid, modifiedCellIdxs] = GetCroppedObstacleGrid(obj, croppedArea)
            [grid, modifiedCellIdxs] = GetRosGrid(obj, obj.ObstacleGrid, croppedArea);
            grid = grid * 100;
        end

        function [grid, modifiedCellIdxs] = GetRosGrid(obj, grid, croppedArea)
            % indices of cells that were updated
            modifiedCellIdxs = GetLinearIndices(obj, croppedArea);
            % cropped area of grid
            grid = getOccupancy(grid, croppedArea, 'grid');
        end

        function linIdxs = GetLinearIndices(obj, croppedArea)
            linIdxs = sub2ind([obj.Width, obj.Height], croppedArea(:,1), croppedArea(:, 2));
        end

        % Returns a costmap of the cropped area that uses the low/med/high
        % probabilistic grids and converts their probabilistic cell values
        % into cost
        function [costmap, modifiedIdxs] = GetCostmap(obj, croppedArea)
            % convert from occupancyMap to matrix
            lowTraffGrid = getOccupancy(obj.LowTraffGrid);
            medTraffGrid = getOccupancy(obj.MediumTraffGrid);
            highTraffGrid = getOccupancy(obj.HighTraffGrid);

            % combine grids and assign respective costs
            costmap = lowTraffGrid * obj.LowTraffCost + ...
                        medTraffGrid * obj.MediumTraffCost + ...
                        highTraffGrid * obj.HighTraffCost;

            modifiedIdxs = GetLinearIndices(obj, croppedArea);
            costmap = costmap(modifiedIdxs);
        end
    end
end