% A class for logging data to a file
classdef Logger

    properties
        mPath;
    end

    properties (Constant)
        LOG_PATH = "../out/";
    end

    methods

        % Creates a logger object with the given filename
        % Not providing a filename will generate a random one
        function self = Logger(fname)
            % Generate random name if not given
            if nargin < 1
                fname = string(java.util.UUID.randomUUID);
            end

            self.mPath = strcat(self.LOG_PATH, fname, ".csv");

            % Create log directory if it doesn't exist
            if ~isfolder(self.LOG_PATH)
                mkdir(self.LOG_PATH);
            end

            % Delete file if it already exists
            if isfile(self.mPath)
                delete(self.mPath);
            end

        end

        % Logs the given data to the file
        function log(self, data)
            writecell(data, self.mPath, 'WriteMode', 'append');
        end

    end

end
