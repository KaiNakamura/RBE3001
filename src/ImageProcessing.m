% Stores utility functions for image processing
classdef ImageProcessing

    properties (Constant)
        OUT_PATH = "../out/";
    end

    methods (Static)

        function saveImage(image, imageTitle)
            imwrite(image, strcat(ImageProcessing.OUT_PATH, imageTitle, ".png"));
        end

        function image = equalizeBrightness(image)
            image = histeq(image, 1024);
        end

        function createBrightnessHistogram(image, imageTitle)
            % Save original image
            ImageProcessing.saveImage(image, imageTitle);

            % Show brightness histogram
            figure;
            imhist(image);
            plotTitle = strcat("Histogram of ", imageTitle);
            saveas(gcf, strcat(ImageProcessing.OUT_PATH, plotTitle, ".png"));
        end

        function image = applyMask(image, mask)
            image = bsxfun(@times, image, cast(mask, "like", image));
        end

        function bw = applyPostProcessing(bw)
        end

        % Faster algorithm for finding centroids, used for live tracking
        function centroids = getCentroidsOfColor(image, color)
            [bw, ~] = ImageProcessing.createColorMask(image, color);
            bw = imdilate(bw, strel("square", 3)); % Dilate image
            bw = imerode(bw, strel("square", 10)); % Erode image
            bw = ImageProcessing.applyPostProcessing(bw);
            stats = regionprops(bw, "centroid");
            centroids = cat(1, stats.Centroid);
        end

        function [centroids, radii, colors] = getCircles(image)
            [~, image] = ImageProcessing.createAllColorMask(image);
            [centroids, radii, ~] = imfindcircles(image, [18 33], "ObjectPolarity", "bright", "Sensitivity", 0.96);
            colors = cell(height(centroids), 1);

            for i = 1:height(centroids)
                [columnsInImage, rowsInImage] = meshgrid(1:width(image), 1:height(image));
                circleMask = (rowsInImage - centroids(i, 2)) .^ 2 + (columnsInImage - centroids(i, 1)) .^ 2 <= radii(i) .^ 2;
                maskedImage = ImageProcessing.applyMask(image, circleMask);
                colors{i} = Color.getClosestColor(maskedImage);
            end

        end

        function image = drawCircles(image, centroids, radii, colors)

            for i = 1:height(centroids)
                centroid = centroids(i, :);
                radius = radii(i);
                color = colors{i};
                image = insertMarker(image, centroid, "s", "Color", color.RGB, "Size", int32(radius));
                image = insertText(image, centroid - [radius, radius], color.name, "BoxColor", color.RGB);
            end

        end

        function [BW, maskedImage] = createColorMask(image, color)

            if strcmpi(color, "all")
                [BW, maskedImage] = ImageProcessing.createAllColorMask(image);
            elseif strcmpi(color, "royg")
                [BW, maskedImage] = ImageProcessing.createROYGMask(image);
            elseif strcmpi(color, "red")
                [BW, maskedImage] = ImageProcessing.createRedMask(image);
            elseif strcmpi(color, "orange")
                [BW, maskedImage] = ImageProcessing.createOrangeMask(image);
            elseif strcmpi(color, "yellow")
                [BW, maskedImage] = ImageProcessing.createYellowMask(image);
            elseif strcmpi(color, "green")
                [BW, maskedImage] = ImageProcessing.createGreenMask(image);
            elseif strcmpi(color, "gray")
                [BW, maskedImage] = ImageProcessing.createGrayMask(image);
            else
                error(strcat("Color not found with name: ", color));
            end

        end

        % Mask for red, orange, yellow, green, and gray
        function [BW, maskedRGBImage] = createAllColorMask(RGB)
            [roygBW, ~] = ImageProcessing.createROYGMask(RGB);
            [grayBW, ~] = ImageProcessing.createGrayMask(RGB);
            BW = roygBW | grayBW;
            maskedRGBImage = ImageProcessing.applyMask(RGB, BW);
        end

        % Mask for just red, orange, yellow, and green
        function [BW, maskedRGBImage] = createROYGMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 04-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.932;
            channel1Max = 0.575;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.154;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.6;
            channel3Max = 1.000;

            % Create mask based on chosen histogram thresholds
            sliderBW = ((I(:, :, 1) >= channel1Min) | (I(:, :, 1) <= channel1Max)) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

        function [BW, maskedRGBImage] = createRedMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 07-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.827;
            channel1Max = 0.056;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.179;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.000;
            channel3Max = 0.927;

            % Create mask based on chosen histogram thresholds
            sliderBW = ((I(:, :, 1) >= channel1Min) | (I(:, :, 1) <= channel1Max)) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

        function [BW, maskedRGBImage] = createOrangeMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 07-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.082;
            channel1Max = 0.134;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.179;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.000;
            channel3Max = 0.927;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:, :, 1) >= channel1Min) & (I(:, :, 1) <= channel1Max) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

        function [BW, maskedRGBImage] = createYellowMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 07-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.135;
            channel1Max = 0.214;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.139;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.000;
            channel3Max = 0.982;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:, :, 1) >= channel1Min) & (I(:, :, 1) <= channel1Max) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

        function [BW, maskedRGBImage] = createGreenMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 07-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.179;
            channel1Max = 0.531;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.179;
            channel2Max = 1.000;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.000;
            channel3Max = 0.927;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:, :, 1) >= channel1Min) & (I(:, :, 1) <= channel1Max) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

        function [BW, maskedRGBImage] = createGrayMask(RGB)
            %  Threshold RGB image using auto-generated code from colorThresholder app.
            %  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
            %  auto-generated code from the colorThresholder app. The colorspace and
            %  range for each channel of the colorspace were set within the app. The
            %  segmentation mask is returned in BW, and a composite of the mask and
            %  original RGB images is returned in maskedRGBImage.

            % Auto-generated by colorThresholder app on 04-Oct-2023
            %------------------------------------------------------

            % Convert RGB image to chosen color space
            I = rgb2hsv(RGB);

            % Define thresholds for channel 1 based on histogram settings
            channel1Min = 0.000;
            channel1Max = 1.000;

            % Define thresholds for channel 2 based on histogram settings
            channel2Min = 0.000;
            channel2Max = 0.286;

            % Define thresholds for channel 3 based on histogram settings
            channel3Min = 0.612;
            channel3Max = 0.790;

            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:, :, 1) >= channel1Min) & (I(:, :, 1) <= channel1Max) & ...
                (I(:, :, 2) >= channel2Min) & (I(:, :, 2) <= channel2Max) & ...
                (I(:, :, 3) >= channel3Min) & (I(:, :, 3) <= channel3Max);
            BW = sliderBW;

            % Initialize output masked image based on input image.
            maskedRGBImage = RGB;

            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW, [1 1 3])) = 0;

        end

    end

end
