% (c) 2023 Robotics Engineering Department, WPI
% Motor Class for the Dynamixel (X-Series) XM430-W350
% The OpenManipulator-X arm consists of five of these (four joints + gripper)
% This class/file should not need to be modified in any way for RBE 3001

classdef DX_XM430_W350
    % Constants
    properties (Constant)
        LIB_NAME = 'libdxl_x64_c'; % Library for Linux, change if other OS
        BAUDRATE = 1000000;
        PROTOCOL_VERSION = 2.0;
        COMM_SUCCESS = 0;
        COMM_TX_FAIL = -1001;

        % Control Table Constants: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
        DRIVE_MODE = 10; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode10
        OPR_MODE = 11; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % Control Table of RAM Area: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area
        TORQUE_ENABLE = 64; % Enable Torque Control
        LED = 65; % Turn LED on/off
        GOAL_CURRENT = 102; % Set/Get goal current
        GOAL_VELOCITY = 104; % Set/Get goal velocity
        PROF_ACC = 108; % set acceleration time for trapezoid profile
        PROF_VEL = 112; % set profile time for trapezoid profile
        GOAL_POSITION = 116; % Set/Get goal position
        CURR_CURRENT = 126; % Get current current
        CURR_VELOCITY = 128; % Get current velocity
        CURR_POSITION = 132; % Get current position

        % message lengths in bytes
        DRIVE_MODE_LEN = 1;
        VEL_PROF = 0;
        TIME_PROF = 4;
        OPR_MODE_LEN = 1;
        CURR_CNTR_MD = 0;
        VEL_CNTR_MD = 1;
        POS_CNTR_MD = 3;
        EXT_POS_CNTR_MD = 4;
        CURR_POS_CNTR_MD = 5;
        PWM_CNTR_MD = 16;
        TORQUE_ENABLE_LEN = 1;
        LED_LEN = 1;
        PROF_ACC_LEN = 4;
        PROF_VEL_LEN = 4;
        CURR_LEN = 2;
        VEL_LEN = 4;
        POS_LEN = 4;

        % Unit Conversions
        MS_PER_S = 1000;
        TICKS_PER_ROT = 4096;
        TICK_POS_OFFSET = DX_XM430_W350.TICKS_PER_ROT / 2; % position value for a joint angle of 0 (2048 for this case)
        TICKS_PER_DEG = DX_XM430_W350.TICKS_PER_ROT / 360;
        TICKS_PER_ANGVEL = 1 / (0.229 * 6); % 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
        TICKS_PER_mA = 1/2.69; % 1 tick = 2.69 mA
    end

    properties
        % Variables
        id; % Dynamixel id
        device_name;
        port_num;
    end

    methods
        % Constructor to set up constants and connect via serial
        function self = DX_XM430_W350(id, device_name)
            self.id = id;
            self.device_name = device_name;

            % Load Libraries
            if ~libisloaded(self.LIB_NAME)
                [notfound, warnings] = loadlibrary(self.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            self.port_num = portHandler(self.device_name);

            self.startConnection();
        end

        % Attempts to connect via serial
        function startConnection(self)
            packetHandler();

            % Open port
            if (~openPort(self.port_num))
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (~setBaudRate(self.port_num, self.BAUDRATE))
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

        end

        % Closes serial connection
        function stopConnection(self)
            closePort(self.port_num)
        end

        % Gets the joint readings (position, velocity, and current) of the motor
        % readings [1x3 double] - The joint positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointReadings(self)
            readings = zeros(1, 3);

            cur_pos = double(self.readWriteByte(self.POS_LEN, self.CURR_POSITION));
            cur_vel = double(self.readWriteByte(self.VEL_LEN, self.CURR_VELOCITY));
            cur_curr = double(self.readWriteByte(self.CURR_LEN, self.CURR_CURRENT));

            cur_pos = (cur_pos - self.TICK_POS_OFFSET) / self.TICKS_PER_DEG;
            cur_vel = cur_vel / self.TICKS_PER_ANGVEL;
            cur_curr = cur_curr / self.TICKS_PER_mA;

            readings(1) = cur_pos;
            readings(2) = cur_vel;
            readings(3) = cur_curr;
        end

        % Commands the motor to go to the desired angle
        % angle [double] - the angle in degrees for the motor to go to
        function writePosition(self, angle)
            position = mod(round(angle * self.TICKS_PER_DEG + self.TICK_POS_OFFSET), self.TICKS_PER_ROT);
            self.readWriteByte(self.POS_LEN, self.GOAL_POSITION, position);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint.
        % time [double] - total profile time in s. If 0, the profile will be disabled.
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
        function writeTime(self, time, acc_time)

            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * self.MS_PER_S;
            acc_time_ms = acc_time * self.MS_PER_S;

            self.readWriteByte(self.PROF_ACC_LEN, self.PROF_ACC, acc_time_ms);
            self.readWriteByte(self.PROF_VEL_LEN, self.PROF_VEL, time_ms);
        end

        % Commands the motor to go at the desired angular velocity
        % velocity [double] - the angular velocity in deg/s for the motor to go at
        function writeVelocity(self, velocity)
            velTicks = velocity * self.TICKS_PER_ANGVEL;
            self.readWriteByte(self.VEL_LEN, self.GOAL_VELOCITY, velTicks);
        end

        % Supplies the motor with the desired current
        % current [double] - the current in mA for the motor to be supplied with
        function writeCurrent(self, current)
            currentInTicks = current * self.TICKS_PER_mA;
            self.readWriteByte(self.CURR_LEN, self.GOAL_CURRENT, currentInTicks);
        end

        % Sets position holding on or off
        % enable [boolean] - true to enable torque to hold last set position, false to disable
        function toggleTorque(self, enable)
            self.readWriteByte(self.TORQUE_ENABLE_LEN, self.TORQUE_ENABLE, enable);
        end

        % Sets motor LED on or off
        % enable [boolean] - true to enable the LED, false to disable
        function toggleLED(self, enable)
            self.readWriteByte(self.LED_LEN, self.LED, enable);
        end

        % Change the operating mode:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function setOperatingMode(self, mode)

            % Save current motor state to go back to that when done
            currentMode = self.readWriteByte(self.TORQUE_ENABLE_LEN, self.TORQUE_ENABLE);

            self.toggleTorque(false);

            switch mode
                case {'current', 'c'}
                    writeMode = self.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = self.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = self.POS_CNTR_MD;
                case {'ext position', 'ep'}
                    writeMode = self.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'}
                    writeMode = self.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'}
                    writeMode = self.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350 class.", mode)
            end

            self.readWriteByte(self.OPR_MODE_LEN, self.OPR_MODE, writeMode);
            self.toggleTorque(currentMode);
        end

        % Verifies if packet was sent/recieved properly, throws an error if not
        % addr [integer] - address of control table index of the last read/write
        % msg [integer] - the message (in bytes) sent with the last read/write
        % msg is an optional parameter. Do not provide if the last serial communication was a read.
        function checkPacket(self, addr, msg)
            dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION);
            packetError = (dxl_comm_result ~= self.COMM_SUCCESS) || (dxl_error ~= 0);

            if exist("msg", "var") && packetError
                fprintf('[msg] %f\n', msg)
            end

            if dxl_comm_result ~= self.COMM_SUCCESS
                fprintf('[addr:%d] %s\n', addr, getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                error("Communication Error: See above.")
            elseif dxl_error ~= 0
                fprintf('[addr:%d] %s\n', addr, getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                error("Recieved Error Packet: See above.")
            end

        end

        % Reads or Writes the message of length n from/to the desired address
        % ignore the 2s complement stuff, we hadn't figured out typecast yet :)
        % n [integer] - The size in bytes of the message
        % (1 for most settings, 2 for current, 4 for velocity/position)
        % addr [integer] - address of control table index to read from or write to
        % msg [integer] - the message (in bytes) to be sent with the write
        % msg is an optional parameter. Do not provide if trying to read from an address.
        function result = readWriteByte(self, n, addr, msg)

            if exist("msg", "var") % write if msg variable was provided
                msg = round(msg);

                switch n % call method to write a certain number of bytes
                    case {1}
                        msg = typecast(int8(msg), 'uint8');
                        write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg);
                    case {2}
                        msg = typecast(int16(msg), 'uint16');
                        write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg);
                    case {4}
                        msg = typecast(int32(msg), 'uint32');
                        write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg);
                    otherwise
                        error("'%s' is not a valid number of bytes to write.\n", n);
                end

                self.checkPacket(addr, msg);
            else % read if msg variable was not provided

                switch n % call method to read a certain number of bytes
                    case {1}
                        result = read1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr);
                        result = typecast(uint8(result), 'int8');
                    case {2}
                        result = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr);
                        result = typecast(uint16(result), 'int16');
                    case {4}
                        result = read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr);
                        result = typecast(uint32(result), 'int32');
                    otherwise
                        error("'%s' is not a valid number of bytes to read.\n", n);
                end

                self.checkPacket(addr);
            end

        end

    end % end methods

end % end class
