% (c) 2023 Robotics Engineering Department, WPI
% Class for OpenManipulator-X arm
% Used to abstract serial connection and read/write methods from Robot class
% This class/file should not need to be modified in any way for RBE 3001

classdef OM_X_arm < handle

    properties
        % DX_XM430_W350 Servos
        motor2_offset = -45; % Example offset, adjust as needed
        motor3_offset = -48;
        motorsNum;
        motorIDs;
        gripperID;
        motors; % 4 Motors, joints 1-4
        gripper; % Gripper end-effector

        % Serial Communication variables
        port_num;
        groupwrite_num;
        groupread_num;
        deviceName;
    end

    methods
        % Constructor to create constants and connect via serial
        % Fun fact, only the motor IDs are needed for bulk read/write,
        % so no instances of the DX_XM430_W350 class are required for the joints.
        % We still provide them (see self.motors) because we found them useful for
        % controlling the motors individually on occasion.
        function self = OM_X_arm()

        end

        % Isolated connection to seperate function so arm code can be used without
        % actually connecting to the arm
        function connect(self)
            % Load Libraries
            if ~libisloaded(DX_XM430_W350.LIB_NAME)
                [notfound, warnings] = loadlibrary(DX_XM430_W350.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            self.motorsNum = 4;
            self.motorIDs = [11, 12, 13, 14];
            self.gripperID = 15;

            % Find serial port and connect to it
            try
                devices = serialportlist();
                ttyDevs = devices(contains(devices, "/dev/ttyUSB"));
                self.deviceName = convertStringsToChars(ttyDevs(1));
            catch exception
                error("Failed to connect via serial, no devices found.")
            end

            self.port_num = portHandler(self.deviceName);

            % Open port
            if (~openPort(self.port_num))
                unloadlibrary(DX_XM430_W350.LIB_NAME);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (~setBaudRate(self.port_num, DX_XM430_W350.BAUDRATE))
                unloadlibrary(DX_XM430_W350.LIB_NAME);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

            self.groupwrite_num = groupBulkWrite(self.port_num, DX_XM430_W350.PROTOCOL_VERSION);
            self.groupread_num = groupBulkRead(self.port_num, DX_XM430_W350.PROTOCOL_VERSION);

            % Create array of motors
            for i = 1:self.motorsNum
                self.motors = [self.motors; DX_XM430_W350(self.motorIDs(i), self.deviceName)];
            end

            % Create Gripper and set operating mode/torque
            self.gripper = DX_XM430_W350(self.gripperID, self.deviceName);
            self.gripper.setOperatingMode('p');
            self.gripper.toggleTorque(true);

            enable = 1;
            disable = 0;

            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, disable);
            self.bulkReadWrite(DX_XM430_W350.DRIVE_MODE_LEN, DX_XM430_W350.DRIVE_MODE, DX_XM430_W350.TIME_PROF);
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);

        end

        % Reads or Writes the message of length n from/to the desired address for all joints
        % result [integer 1x4] - the result of a bulk read, empty if bulk write
        % n [integer] - The size in bytes of the message
        % (1 for most settings, 2 for current, 4 for velocity/position)
        % addr [integer] - address of control table index to read from or write to
        % msgs [integer 1x4] - the messages (in bytes) to send to each joint, respectively
        % msgs is optional. Do not provide if trying to read.
        % msgs can also be an integer, where the same message will be sent to all four joints.
        function result = bulkReadWrite(self, n, addr, msgs)

            if ~exist("msgs", "var") % Bulk Read if msgs does not exist
                groupBulkReadClearParam(self.groupread_num);

                result = zeros(1, 4);

                for id = self.motorIDs
                    dxl_addparam_result = groupBulkReadAddParam(self.groupread_num, id, addr, n);

                    if dxl_addparam_result ~= true
                        fprintf('[ID:%03d ADDR:%d] groupBulkRead addparam failed\n', id, addr);
                        return;
                    end

                end

                groupBulkReadTxRxPacket(self.groupread_num);
                dxl_comm_result = getLastTxRxResult(self.port_num, DX_XM430_W350.PROTOCOL_VERSION);

                if dxl_comm_result ~= DX_XM430_W350.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(DX_XM430_W350.PROTOCOL_VERSION, dxl_comm_result));
                end

                for i = 1:self.motorsNum
                    id = self.motorIDs(i);
                    dxl_getdata_result = groupBulkReadIsAvailable(self.groupread_num, id, addr, n);

                    if dxl_getdata_result ~= true
                        fprintf('[ID:%03d ADDR:%d] groupBulkRead getdata failed\n', id, addr);
                        return;
                    end

                    readBits = groupBulkReadGetData(self.groupread_num, id, addr, n);

                    switch (n)
                        case {1}
                            result(i) = typecast(uint8(readBits), 'int8');
                        case {2}
                            result(i) = typecast(uint16(readBits), 'int16');
                        case {4}
                            result(i) = typecast(uint32(readBits), 'int32');
                        otherwise
                            error("'%s' is not a valid number of bytes to read.\n", n);
                    end

                    % Subtract offsets after reading for motors 2 and 3
                    if id == self.motorIDs(2)
                        result(i) = result(i) - self.motor2_offset;
                    elseif id == self.motorIDs(3)
                        result(i) = result(i) - self.motor3_offset;
                    end

                end

            else % Bulk Write if msgs exists

                if length(msgs) == 1
                    msgs = repelem(msgs, 4);
                end

                % Add offset to motor 3 joint angle before writing
                msgs(2) = msgs(2) + self.motor2_offset;
                msgs(3) = msgs(3) + self.motor3_offset;

                groupBulkWriteClearParam(self.groupwrite_num);

                for i = 1:length(self.motorIDs)
                    id = self.motorIDs(i);

                    switch (n)
                        case {1}
                            msg = typecast(int8(msgs(i)), 'uint8');
                        case {2}
                            msg = typecast(int16(msgs(i)), 'uint16');
                        case {4}
                            msg = typecast(int32(msgs(i)), 'uint32');
                        otherwise
                            error("'%s' is not a valid number of bytes to write.\n", n);
                    end

                    dxl_addparam_result = groupBulkWriteAddParam(self.groupwrite_num, id, addr, n, msg, n);

                    if dxl_addparam_result ~= true
                        fprintf('[ID:%03d ADDR:%d MSG:%d] groupBulkWrite addparam failed\n', id, addr, msgs(i));
                        return;
                    end

                end

                groupBulkWriteTxPacket(self.groupwrite_num);
                dxl_comm_result = getLastTxRxResult(self.port_num, DX_XM430_W350.PROTOCOL_VERSION);

                if dxl_comm_result ~= DX_XM430_W350.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(DX_XM430_W350.PROTOCOL_VERSION, dxl_comm_result));
                end

            end

        end

    end % end methods

end % end class
