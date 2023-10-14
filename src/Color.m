classdef Color

    properties (Constant)
        RED = Color("red", [190 108 124], [0, -200, 0, 90]);
        ORANGE = Color("orange", [210 194 147], [70, -200, 0, 90]);
        YELLOW = Color("yellow", [219 214 141], [140, -200, 0, 90]);
        GREEN = Color("green", [133 195 163], [70, 200, 0, 90]);
        GRAY = Color("gray", [162 175 180], [140, 200, 0, 90]);
        ALL = {Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.GRAY};
    end

    properties
        name;
        RGB;
        deliveryZone;
    end

    methods

        function self = Color(name, RGB, deliveryZone)
            self.name = name;
            self.RGB = RGB;
            self.deliveryZone = deliveryZone;
        end

    end

    methods (Static)

        % Takes an image and returns the color that most closely matches the image
        function closestColor = getClosestColor(RGB)
            closestColor = Color.ALL{1};
            maxPixelCount = -1;

            for i = 1:length(Color.ALL)
                color = Color.ALL{i};
                [bw, ~] = ImageProcessing.createColorMask(RGB, color.name);
                pixelCount = height(find(bw));

                if pixelCount > maxPixelCount
                    closestColor = color;
                    maxPixelCount = pixelCount;
                end

            end

        end

    end

end
