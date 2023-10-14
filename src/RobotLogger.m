% Class for logging robot data to a file
classdef RobotLogger

    properties
        mLogger Logger;
        mRobot Robot;
    end

    methods

        % Creates a logger object with the given filename
        % extraDataLabels is a cell array of strings that will be used as labels for the extra data
        % If fname is negative, the filename will be a random uuid
        function self = RobotLogger(robot, fname, extraDataLabels)

            if fname == ""
                self.mLogger = Logger();
            else
                self.mLogger = Logger(fname);
            end

            self.mRobot = robot;
            % Log the labels
            self.mLogger.log([{"q1_pos", "q2_pos", "q3_pos", "q4_pos", "q1_vel", "q2_vel", "q3_vel", "q4_vel", "q1_target", "q2_target", "q3_target", "q4_target"}, extraDataLabels]);
        end

        % Logs robot state as well as any additional data
        function log(self, extraData)

            if nargin < 2
                extraData = {};
            end

            pv = self.mRobot.measured_js(1, 1);
            position = pv(1, :);
            velocity = pv(2, :);
            target_position = self.mRobot.mGoals;
            self.mLogger.log([{position, velocity, target_position}, extraData]);
        end

    end

end
