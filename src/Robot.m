% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mL2Angle; % Stores the offset angle of link 2 when in the zero position
        mTheta; % Stores the symbolic joint position angles (degrees)
        mDHTable; % Stores the robot DH table
        mGoals; % Stores the angles (degrees) for the target setpoints of each of the joints in a 1x4 array
        mT01; % Stores the transformation from frame 0 to 1
        mT02; % Stores the transformation from frame 0 to 2
        mT03; % Stores the transformation from frame 0 to 3
        mT04; % Stores the transformation from frame 0 to 4
        mT0ee; % Stores the transformation from frame 0 to the end effector
        mModel; % Stores the stick model for the robot that shows frames, joints, and links
        mConnected; % Stores whether or not the robot is connected to the computer
        mJacob; % Stores the symbolic Jacobian matrix
        mCheckSingularities; % Stores whether or not to check for singularities while moving the robot. modified externally
        fk_syms_func; % Stores a function handle for the symbolic forward kinematics
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot(connect)
            % Assume connection if not specified
            if nargin < 1
                connect = true;
            end

            % Robot Dimensions
            self.mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.mL2Angle = atand(self.mOtherDim(2) / self.mOtherDim(1));

            self.mConnected = false;
            self.mGoals = zeros(1, 4); % Initialize goals to 0

            syms theta [1 4]
            self.mTheta = theta;
            self.mDHTable = [0 self.mDim(1) 0 0; ...
                                 theta1 self.mDim(2) 0 -90; ...
                                 (theta2 + self.mL2Angle - 90) 0 self.mDim(3) 0; ...
                                 (theta3 + 90 - self.mL2Angle) 0 self.mDim(4) 0; ...
                                 theta4 0 self.mDim(5) 0];
            self.dh2fk(self.mDHTable); % Generate the symbolic forward kinematics matrices

            % This might make the code run faster, idk.
            self.mT01 = simplify(self.mT01);
            self.mT02 = simplify(self.mT02);
            self.mT03 = simplify(self.mT03);
            self.mT04 = simplify(self.mT04);
            self.mT0ee = simplify(self.mT0ee);

            self.mJacob = simplify(self.getSymJacobian());
            self.mCheckSingularities = true;

            self.mModel = Model();

            fk_matrix = Kinematics.dh_table_transform(self.mDHTable);
            self.fk_syms_func = matlabFunction(simplify(fk_matrix));

            if connect
                self.connect();
            else
                disp("Not connecting to robot. Some behaviors are simplified or may not work.");
            end

        end

        function connect(self)
            % Call super method to connect to the robot
            self.connect@OM_X_arm();

            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);

            self.mConnected = true;
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            self.mGoals = goals;
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);

            if self.mCheckSingularities
                self.checkHandleSingularity();
            end

            if self.mConnected
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            else
                % if not connected, show the live plot
                self.live_plot_arm();
            end

        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
        function writeTime(self, time, acc_time)

            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            if self.mConnected
                self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
                self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
            end

        end

        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)

            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end

        end

        % Sets the gripper to be open
        function openGripper(self)
            self.writeGripper(true)
        end

        % Sets the gripper to be closed
        function closeGripper(self)
            self.writeGripper(false)
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)

            switch mode
                case {'current', 'c'}
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3, 4);

            if self.mConnected
                readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
                readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
            else
                readings(1, :) = self.mGoals;
            end

        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        % Takes a 1x4 array of joint values in degrees to be sent directly to the actuators and
        % bypasses interpolation
        function servo_jp(self, joint_positions)
            self.writeTime(1/20);
            self.writeJoints(joint_positions);
        end

        % Takes a 1x4 array of joint values in degrees and an interpolation time in ms to get there
        function interpolate_jp(self, joint_positions, time)
            self.writeTime(time);
            self.writeJoints(joint_positions);
        end

        % Takes two boolean values, named GETPOS and GETVEL and returns the results for the requested data,
        % and sets the rest to zero. This will be important because if we want to collect position data very fast,
        % we will not want to slow the system down by also acquiring unnecessary velocity data. Returns a 2x4 array
        % that contains current joint positions in degrees (1st row) and/or current joint velocities (2nd row).
        function readings = measured_js(self, GETPOS, GETVEL)
            readings = zeros(2, 4);

            if GETPOS

                if self.mConnected
                    readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
                else
                    readings(1, :) = self.mGoals;
                end

            end

            if GETVEL

                if self.mConnected
                    readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                else
                    readings(2, :) = zeros(1, 4);
                end

            end

        end

        % Returns a 1x4 array that contains current joint set point positions in degrees.
        % If interpolation is being used and is requested during motion, it will return the current intermediate setpoint.
        function setpoints = setpoint_js(self)

            if self.mConnected
                setpoints = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            else
                setpoints = self.mGoals;
            end

        end

        % Returns a 1x4 array that contains the end-of-motion joint setpoint positions in degrees.
        function goals = goal_js(self)
            goals = self.mGoals;
        end

        % This method takes in a 1x4 array corresponding to a row of the DH parameter table for
        % a given link. It then generates and returns the corresponding symbolic 4x4
        % homogeneous transformation matrix.
        function T = dh2mat(~, dh)
            T = Kinematics.dh_transform(dh);
        end

        % Takes in an nx4 array corresponding to the n rows of the full DH parameter table together with T01.
        % It then generates a corresponding symbolic 4x4 homogeneous transformation matrix for the composite
        % transformation by symbolically calculating T01 * T12 * T23 * T34 * T4ee.
        % Each row of the matrix corresponds to the DH parameters (theta, d, a, alpha) for that link
        % (i.e. row 1 corresponds to the DH parameters for link 1)
        function T = dh2fk(self, dh)
            self.mT01 = Kinematics.dh_transform(dh(1, :));
            self.mT02 = self.mT01 * Kinematics.dh_transform(dh(2, :));
            self.mT03 = self.mT02 * Kinematics.dh_transform(dh(3, :));
            self.mT04 = self.mT03 * Kinematics.dh_transform(dh(4, :));
            self.mT0ee = self.mT04 * Kinematics.dh_transform(dh(5, :));
            T = self.mT0ee;
        end

        % Takes n joint values as inputs in the form of an nx1 vector.
        % Returns a 4x4 homogeneous transformation matrix representing the position and orientation of the
        % tip frame with respect to the base frame (i.e. T0ee or TBaseTip).
        % Given a set of n joint values, you can substitute into the symbolic matrix to get a 4x4
        % numeric matrix representing the numeric forward kinematics for the given set of joint
        % values. For link lengths, use the values provided in Lab 1 for the robot.
        function T = fk3001(self, joint_positions)
            % T = Kinematics.dh_table_transform(eval(subs(self.mDHTable, self.mTheta, joint_positions)));
            T = self.fk_syms_func(joint_positions(1), joint_positions(2), joint_positions(3), joint_positions(4));
        end

        % Takes data from measured_js() and returns a 4x4 homogeneous transformation
        % matrix based upon the current joint positions in degrees.
        function T = measured_cp(self)
            readings = self.measured_js(true, false);
            T = self.fk3001(readings(1, :));
        end

        % Takes data from setpoint_js() and returns a 4x4 homogeneous transformation
        % matrix based upon the current joint set point positions in degrees.
        % If interpolation is being used, it will return the current intermediate setpoint.
        function T = setpoint_cp(self)
            T = self.fk3001(self.setpoint_js());
        end

        % Takes data from goal_js() and returns a 4x4 homogeneous transformation
        % matrix based upon the commanded end of motion joint set point positions in degrees.
        function T = goal_cp(self)
            T = self.fk3001(self.goal_js());
        end

        % Takes a 1x4 array of joint values and plots a stick model of the arm showing all frames, joints, and links.
        function plot_arm(self, joint_positions, velocity)
            T0 = eye(4);
            T01 = eval(subs(self.mT01, self.mTheta, joint_positions));
            T02 = eval(subs(self.mT02, self.mTheta, joint_positions));
            T03 = eval(subs(self.mT03, self.mTheta, joint_positions));
            T04 = eval(subs(self.mT04, self.mTheta, joint_positions));
            T0ee = eval(subs(self.mT0ee, self.mTheta, joint_positions));

            if (exist("velocity", "var"))
                self.mModel.plot({T0, T01, T02, T03, T04, T0ee}, velocity);
            else
                self.mModel.plot({T0, T01, T02, T03, T04, T0ee});
            end

        end

        % Plots a stick model of the current arm position showing all frames, joints, and links.
        function live_plot_arm(self)
            self.plot_arm(self.setpoint_js());
        end

        % Plots a stick model of the current arm position showing all frames, joints, and links.
        function live_plot_arm_vel(self)
            self.plot_arm(self.setpoint_js(), self.getVelocity());
        end

        % Takes a 1×4 task space vector for x,y,z position (i.e. P0ee) and orientation of the end-
        % effector w.r.t the horizontal plane (alpha).
        % Returns a 1×4 joint space vector for corresponding joint angles (i.e. q1, q2, q3, q4).
        % Calculates the corresponding joint angles to move the robot’s end-effector to specified
        % task space (position & orientation). Throws an error if the desired end-effector pose is
        % unreachable.
        function q = ik3001(self, p)

            if ~self.isValidWorkspacePosition(p(1:3))
                error(strcat("Position ", mat2str(p), " is not in the workspace."));
            end

            % Get joint lengths
            L1 = self.mDim(1) + self.mDim(2);
            L2 = self.mDim(3);
            L3 = self.mDim(4);
            L4 = self.mDim(5);

            % Get the parts of p
            x = p(1, 1);
            y = p(1, 2);
            z = p(1, 3);
            alpha = p(1, 4);

            % Define helpful dimensions
            r = sqrt(x ^ 2 + y ^ 2);
            x24 = r - (L4 * cosd(-alpha));
            z24 = z - L1 - (L4 * sind(-alpha));
            L24Squared = x24 ^ 2 + z24 ^ 2;
            L24 = sqrt(L24Squared);

            % Calculate theta1
            d1 = x / r;
            c1 = sqrt(1 - d1 ^ 2);
            theta1 = zeros(1, 2);
            theta1(1) = atan2d(c1, d1);
            theta1(2) = atan2d(-c1, d1);

            % Calculate beta
            dbeta = (L2 ^ 2 + L24Squared - L3 ^ 2) / (2 * L2 * L24);
            cbeta = sqrt(1 - dbeta ^ 2);
            beta = zeros(1, 2);
            beta(1) = atan2d(cbeta, dbeta);
            beta(2) = atan2d(-cbeta, dbeta);

            % Calculate phi
            dphi = x24 / L24;
            cphi = sqrt(1 - dphi ^ 2);
            phi = zeros(1, 2);
            phi(1) = atan2d(cphi, dphi);
            phi(2) = atan2d(-cphi, dphi);

            % Calculate theta2
            betaAndPhi = combinations(beta, phi);
            theta2 = betaAndPhi{:, 1} - betaAndPhi{:, 2};

            % Calculate theta3
            d3 = -1 * (L2 ^ 2 + L3 ^ 2 - L24Squared) / (2 * L2 * L3);
            c3 = sqrt(1 - d3 ^ 2);
            theta3 = zeros(1, 2);
            theta3(1) = atan2d(c3, d3);
            theta3(2) = atan2d(-c3, d3);

            % Calculate theta4
            theta2AndTheta3 = combinations(theta2, theta3);
            theta4 = (theta2AndTheta3{:, 1} + theta2AndTheta3{:, 2}) + alpha;

            % Calculate possible values for q
            q1 = theta1;
            q2 = -theta2 - self.mL2Angle + 90;
            q3 = theta3 + self.mL2Angle - 90;
            q4 = theta4;
            candidates = table2array(combinations(q1, q2, q3, q4));

            for i = 1:height(candidates)
                candidate = candidates(i, :);

                if ~Robot.isValidJointPosition(candidate)
                    continue;
                end

                fk = self.fk3001(candidate);
                actual = fk(1:3, 4);
                target = [x; y; z];
                distance = norm(target - actual);

                if distance < 1
                    q = candidate;
                    return;
                end

            end

            error(strcat("Joint positions could not be found for ", mat2str(p)));

        end

        % Takes a 1x3 array of coordinates and returns whether or not the coordinates are in the workspace
        function valid = isValidWorkspacePosition(self, p)
            % The workspace is just a sphere centered around joint 2, so this is an easy check
            workspaceOrigin = [0, 0, self.mDim(1) + self.mDim(2)];
            workspaceRadius = self.mDim(3) + self.mDim(4) + self.mDim(5);
            valid = norm(p - workspaceOrigin) <= workspaceRadius;
        end

        % Moves the joints to desired positions over the specified duration
        function moveToJointPositions(self, joint_positions, duration)
            self.interpolate_jp(joint_positions, duration);
            pause(duration);
        end

        % Moves the robot to the home position over the specified duration
        function moveToHome(self, duration)
            self.moveToJointPositions([0, 0, 0, 0], duration);
        end

        % Takes a 1×4 task space vector for x,y,z position (i.e. P0ee) and orientation of the end-
        % effector w.r.t the horizontal plane (alpha) and a duration
        % Moves the robot to the desired pose over the specified duration
        % Returns whether or not the robot could reach the pose
        function couldReach = moveToPose(self, pose, duration)

            try
                joint_positions = self.ik3001(pose);
            catch
                couldReach = false;
                return;
            end

            couldReach = true;
            self.moveToJointPositions(joint_positions, duration);
        end

        % Takes in trajectory coefficients as a 4x4 matrix (4 joints as cols, 4 coefficients as rows which are
        % generated from cubic_traj(). Also takes in the total amount of time the trajectory should take. This must
        % be the same duration passed to cubic_traj() to generate the trajectories for each joint.
        % This function will run a while loop, which calculates the current joint poses based on the trajectory
        % coefficients and current time, and commands the robot to go to these with servo_jp(). Within the loop,
        % the function also saves time and joint angle data to a matrix to plot later. This function then returns the
        % saved matrix data.
        function run_trajectory(self, trajectory, duration, is_joint_space, robotLogger)
            a0 = trajectory(1, :);
            a1 = trajectory(2, :);
            a2 = trajectory(3, :);
            a3 = trajectory(4, :);

            if height(trajectory) == 6
                a4 = trajectory(5, :);
                a5 = trajectory(6, :);
            end

            tic; % Start timer

            while toc < duration

                if height(trajectory) == 6
                    % is quintic polynomial
                    if (is_joint_space)
                        q = a0 + a1 * toc + a2 * toc ^ 2 + a3 * toc ^ 3 + a4 * toc ^ 4 + a5 * toc ^ 5;
                        self.servo_jp(q);
                    else
                        p = a0 + a1 * toc + a2 * toc ^ 2 + a3 * toc ^ 3 + a4 * toc ^ 4 + a5 * toc ^ 5;
                        q = self.ik3001(p);
                        self.servo_jp(q);
                    end

                else
                    % is cubic polynomial
                    if (is_joint_space)
                        q = a0 + a1 * toc + a2 * toc ^ 2 + a3 * toc ^ 3;
                        self.servo_jp(q);
                    else
                        p = a0 + a1 * toc + a2 * toc ^ 2 + a3 * toc ^ 3;
                        q = self.ik3001(p);
                        self.servo_jp(q);
                    end

                end

                % If there is a logger, log
                if nargin == 5
                    robotLogger.log({toc})
                end

            end

        end

        function j = getSymJacobian(self)
            x_grad = gradient(self.mT0ee(1, 4), self.mTheta).'; % [dx/dq1, dx/dq2, dx/dq3, dx/dq4]
            y_grad = gradient(self.mT0ee(2, 4), self.mTheta).'; % [dy/dq1, dy/dq2, dy/dq3, dy/dq4]
            z_grad = gradient(self.mT0ee(3, 4), self.mTheta).'; % [dz/dq1, dz/dq2, dz/dq3, dz/dq4]
            Jp = [x_grad; y_grad; z_grad]; % [3x4]

            zhat_q1 = self.mT01(1:3, 3);
            zhat_q2 = self.mT02(1:3, 3);
            zhat_q3 = self.mT03(1:3, 3);
            zhat_q4 = self.mT04(1:3, 3);
            Jw = [zhat_q1, zhat_q2, zhat_q3, zhat_q4]; % [3x4]
            j = [Jp; Jw];
        end

        % This method takes the configuration q (i.e. all of the current joint angles at the time
        % the function is run) and returns the corresponding numeric 6x4 Jacobian matrix.
        function j = jacob3001(self, q)
            j = eval(subs(self.mJacob, self.mTheta, q));
        end

        function is_singularity = isSingularity(self, q)
            j = self.jacob3001(q);
            is_singularity = det(j(1:3, 1:3)) < 1e-3;
        end

        function checkHandleSingularity(self)
            jr = self.setpoint_js();

            if self.isSingularity(jr)
                set(gca, 'Color', 'r'); % set plot to red
                self.live_plot_arm();
                error("Singular configuration detected. Aborting.");
            end

        end

        % o This method takes your configuration 𝑞 (i.e. all of the current joint angles at the time
        %   the function is running) and the vector of instantaneous joint velocities 𝑞̇ as inputs.
        % o It should return the 6x1 vector including the task - space linear velocities 𝑝̇ and angular
        %   velocities 𝜔.
        % o Use 𝑞 as input to the ‘jacob3001()’ method that you developed in Part 2 to
        %   calculate the Jacobian matrix for the current configuration, 𝐽(𝑞) from within this
        %   method.
        % q and dpdt are both 1x4 arrays
        function dpdt = fdk3001(self, q, dqdt)
            jacob = self.jacob3001(q);
            dpdt = jacob * dqdt.';
        end

        function v = getVelocity(self)
            v = self.getJointsReadings();
            pos = v(1, :);
            vel = v(2, :) - [0 61.8300 65.9520 0]; % For some reason there is constant error from readings?
            v = self.fdk3001(pos, vel);
        end

    end % end methods

    methods (Static)
        % Takes a 1x4 array of joint values and returns whether or not the joint position is valid
        % (i.e. None of the joint values exceed the maximum safe range for that joint)
        function valid = isValidJointPosition(q)
            valid = abs(q(1)) <= 180 && abs(q(2)) <= 90 && abs(q(3)) <= 90 && abs(q(4)) <= 90;
        end

    end

end % end class
