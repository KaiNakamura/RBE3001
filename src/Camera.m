classdef Camera < handle

    properties
        params; % Camera Parameters
        cam; % Webcam Object
        cam_pose; % Camera Pose (transformation matrix)
        cam_IS; % Camera Intrinsics
        cam_R; % Camera Rotation Matrix
        cam_T; % Camera Translation Vector
        cam_TForm % Camera Rigid 3D TForm
        imagePoints; % Image points of checkerboard intersections
    end

    methods

        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = Camera.calibrate(); % Run calibration function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
        end

        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraInstrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        % SHUTDOWN shutdown script which clears camera variable
        function shutdown(~)
            clear self.cam;
        end

        % Returns a raw, distorted camera image
        function rawImage = getRawImage(self)
            rawImage = snapshot(self.cam);
        end

        % Undistorts an image and returns the undistorted image and camera intrinsics
        function [undistortedImage, camIntrinsics] = undistortImage(self, rawImage)
            [undistortedImage, camIntrinsics] = undistortFisheyeImage(rawImage, self.params.Intrinsics, "OutputView", "full");
        end

        function [camIntrinsics, pose] = calculateCameraPos(self)
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            rawImage = self.getRawImage();
            % 2. Undistort Image based on params
            [img, camIntrinsics] = self.undistortImage(rawImage);
            % 3. Detect checkerboard in the image
            [self.imagePoints, boardSize] = detectCheckerboardPoints(img, "PartialDetections", false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1) - 1) * 25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(self.imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(self.imagePoints, self.params.WorldPoints, camIntrinsics);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigidtform3d(R', t);

            pose = [R, t'; 0, 0, 0, 1];
        end

        % Get outer corners of the checkerboard from image points
        function corners = getWorkspaceCorners(self)
            topLeft = Camera.getCornerPoint(self.imagePoints(1, :), self.imagePoints(6, :), 1.2);
            topRight = Camera.getCornerPoint(self.imagePoints(37, :), self.imagePoints(34, :), 1.2);
            bottomRight = Camera.getCornerPoint(self.imagePoints(40, :), self.imagePoints(35, :), 1.8);
            bottomLeft = Camera.getCornerPoint(self.imagePoints(4, :), self.imagePoints(7, :), 1.8);
            corners = [topLeft; topRight; bottomRight; bottomLeft];
        end

        % Apply the workspace mask to a provided image
        function image = applyWorkspaceMask(self, image)
            corners = self.getWorkspaceCorners();
            workspaceMask = poly2mask(corners(:, 1)', corners(:, 2)', height(image), width(image));
            image = ImageProcessing.applyMask(image, workspaceMask);
        end

        % Gets a fully processed camera image of the workspace
        function image = getWorkspaceImage(self)
            image = self.getRawImage();
            image = ImageProcessing.equalizeBrightness(image);
            image = self.undistortImage(image);
            image = self.applyWorkspaceMask(image);
        end

        function checkerboardPoints = imagePointsToCheckerboardPoints(self, imagePoints)
            checkerboardPoints = img2world2d(imagePoints, self.getTForm(), self.getCameraInstrinsics());
        end

        function worldPoints = imagePointsToWorldPoints(self, imagePoints)
            checkerboardPoints = self.imagePointsToCheckerboardPoints(imagePoints);
            worldPoints = Camera.checkerboardPointsToWorldPoints(checkerboardPoints);
        end

        function objectPoints = worldPointsToObjectPoints(self, worldPoints, objectHeight)
            cameraHeight = self.cam_T(3);
            objectPoints = zeros(height(worldPoints), 3);

            for i = 1:height(worldPoints)
                point = worldPoints(i, :);
                pointX = point(1);
                pointY = point(2);
                Dx = 400 - pointX;
                Dy = pointY;
                distanceToPoint = sqrt(Dx ^ 2 + Dy ^ 2);
                distanceToObject = distanceToPoint * (objectHeight / cameraHeight);
                dx = distanceToObject * (Dx / distanceToPoint);
                dy = distanceToObject * (Dy / distanceToPoint);
                objectPoints(i, :) = [pointX + dx, pointY - dy, objectHeight];
            end

        end

        function objectPoints = imagePointsToObjectPoints(self, imagePoints, objectHeight)

            if isempty(imagePoints)
                objectPoints = [];
                return;
            end

            worldPoints = self.imagePointsToWorldPoints(imagePoints);
            objectPoints = self.worldPointsToObjectPoints(worldPoints, objectHeight);
        end

    end

    methods (Static)

        function params = calibrate()
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            cameraParamsPath = "../camera_calibration/cameraParams.mat";

            if isfile(cameraParamsPath)
                params = load(cameraParamsPath).cameraParams;
            else

                try
                    disp("Clear surface of any items, then press any key to continue");
                    pause;
                    disp("Calibrating");
                    run("../camera_calibration/CameraCalibration.m");
                    params = cameraParams; %#ok<USENS>
                    save(cameraParamsPath, "cameraParams");
                    disp("Camera calibration complete!");
                catch exception
                    msg = getReport(exception);
                    disp(msg);
                    error("No camera calibration file found. Plese run camera calibration");
                end

            end

        end

        % Gets an outer corner point of a checkerboard
        % p1 is an image point of the checkerboard
        % p2 is an imape point of the checkerboard digonal to p1
        function cornerPoint = getCornerPoint(p1, p2, scale)
            distance = p2 - p1;
            cornerPoint = p1 - (distance * scale);
        end

        function worldPoints = checkerboardPointsToWorldPoints(checkerboardPoints)
            worldPoints = zeros(height(checkerboardPoints), 3);

            for i = 1:height(checkerboardPoints)
                checkerboardPoint = checkerboardPoints(i, 1:2);
                R_0_checker = [0 1 0; 1 0 0; 0 0 -1];
                t_0_checker = [113; -95; 0];
                T_0_check = [R_0_checker, t_0_checker; zeros(1, 3), 1];
                r_pos = (T_0_check \ [checkerboardPoint'; 0; 1]);
                worldPoints(i, :) = r_pos(1:3)';

                % If checkerboard point contains z, add it to world point
                if width(checkerboardPoint) == 3
                    worldPoints(i, :) = worldPoints(i, :) + [0, 0, checkerboardPoint(1, 3)];
                end

            end

        end

    end

end
