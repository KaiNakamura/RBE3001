% Stores utility functions for kinematic calculations
classdef Kinematics

    methods (Static)
        % Generates a rotation matrix for a X axis rotation
        function T = rotx(theta)
            T = [1 0 0 0; 0 cosd(theta) -sind(theta) 0; 0 sind(theta) cosd(theta) 0; 0 0 0 1];
        end

        % Generates a rotation matrix for a Y axis rotation
        function T = roty(theta)
            T = [cosd(theta) 0 sind(theta) 0; 0 1 0 0; -sind(theta) 0 cosd(theta) 0; 0 0 0 1];
        end

        % Generates a rotation matrix for a Z axis rotation
        function T = rotz(theta)
            T = [cosd(theta) -sind(theta) 0 0; sind(theta) cosd(theta) 0 0; 0 0 1 0; 0 0 0 1];
        end

        % Generates a translation matrix
        function T = translate(x, y, z)
            T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
        end

        % Generates a transformation matrix from a DH parameter
        function T = dh_transform(dh_param)
            theta = dh_param(1);
            d = dh_param(2);
            a = dh_param(3);
            alpha = dh_param(4);
            T = Kinematics.rotz(theta) * Kinematics.translate(0, 0, d) * Kinematics.translate(a, 0, 0) * Kinematics.rotx(alpha);
        end

        % Generates a transformation matrix from a DH parameter set
        % The first dimension of the matrix is the joint number
        % The second dimension is the DH parameters (theta, d, a, alpha)
        function T = dh_table_transform(dh)
            T = eye(4);

            for i = 1:size(dh, 1)
                T = T * Kinematics.dh_transform(dh(i, :));
            end

        end

    end

end
