classdef Model

    properties (Constant)
        X_LIM = [-400, 400];
        Y_LIM = [-400, 400];
        Z_LIM = [0, 600];
        QUIVER_SCALE = 0.5;
    end

    properties
        mPositionsPlot; % plot3d for joint positions
        mXAxesQuiver; % quiver3 for the x axis of each frame
        mYAxesQuiver; % quiver3 for the y axis of each frame
        mZAxesQuiver; % quiver3 for the z axis of each frame
        mVQuiver; % quiver3 for the velocity of the end effector
    end

    methods

        function self = Model()
            figure;

            % Initialize quivers
            self.mXAxesQuiver = quiver3(0, 0, 0, 0, 0, 0, Model.QUIVER_SCALE, 'g', 'LineWidth', 2);
            hold on
            self.mYAxesQuiver = quiver3(0, 0, 0, 0, 0, 0, Model.QUIVER_SCALE, 'b', 'LineWidth', 2);
            hold on;
            self.mZAxesQuiver = quiver3(0, 0, 0, 0, 0, 0, Model.QUIVER_SCALE, 'r', 'LineWidth', 2);
            hold on;
            self.mVQuiver = quiver3(0, 0, 0, 0, 0, 0, Model.QUIVER_SCALE / 2, 'p', 'LineWidth', 2);
            hold on;

            % Initialize plot
            self.mPositionsPlot = plot3(0, 0, 0, 'ko-', 'LineWidth', 1.5);
            rotate3d on;
            view(135, 30);
            xlabel("X (mm)");
            ylabel("Y (mm)");
            zlabel("Z (mm)");
            xlim(Model.X_LIM);
            ylim(Model.Y_LIM);
            zlim(Model.Z_LIM);
            hold off;
        end

        % Takes an 1xn cell array of transformations where n is the number of transformations and
        % plots a stick model of the arm showing all frames, joints, and links.
        % To call this method use model.plot({T01, T02, ..., T0N});
        % velocity is an optional [3x1] vector with the velocity of the end effector
        function plot(self, transformations, velocity)
            positions = zeros(width(transformations), 3);
            xAxes = zeros(width(transformations), 3);
            yAxes = zeros(width(transformations), 3);
            zAxes = zeros(width(transformations), 3);

            for i = 1:width(transformations)
                % Get current transformation, rotation, and point
                T = transformations{i};
                R = T(1:3, 1:3);
                P = T(1:3, 4);

                % Add position to array
                positions(i, :) = P.';

                % Add each axis to corresponding array
                xAxes(i, :) = R(:, 1);
                yAxes(i, :) = R(:, 2);
                zAxes(i, :) = R(:, 3);
            end

            % Update x axes
            set(self.mXAxesQuiver, ...
                'XData', positions(:, 1), ...
                'YData', positions(:, 2), ...
                'ZData', positions(:, 3), ...
                'UData', xAxes(:, 1), ...
                'VData', xAxes(:, 2), ...
                'WData', xAxes(:, 3));

            % Update y axes
            set(self.mYAxesQuiver, ...
                'XData', positions(:, 1), ...
                'YData', positions(:, 2), ...
                'ZData', positions(:, 3), ...
                'UData', yAxes(:, 1), ...
                'VData', yAxes(:, 2), ...
                'WData', yAxes(:, 3));

            % Update z axes
            set(self.mZAxesQuiver, ...
                'XData', positions(:, 1), ...
                'YData', positions(:, 2), ...
                'ZData', positions(:, 3), ...
                'UData', zAxes(:, 1), ...
                'VData', zAxes(:, 2), ...
                'WData', zAxes(:, 3));

            % Update velocity
            if (exist("velocity", "var"))
                set(self.mVQuiver, ...
                    'XData', positions(end, 1), ...
                    'YData', positions(end, 2), ...
                    'ZData', positions(end, 3), ...
                    'UData', velocity(1), ...
                    'VData', velocity(2), ...
                    'WData', velocity(3), ...
                    "Color", "k");
            else
                set(self.mVQuiver, ...
                    "Color", "none");
            end

            % Update position plot
            set(self.mPositionsPlot, 'XData', positions(:, 1), 'YData', positions(:, 2), 'ZData', positions(:, 3));
            drawnow;
        end

    end

end
