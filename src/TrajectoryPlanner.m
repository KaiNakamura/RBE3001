classdef TrajectoryPlanner

    methods (Static)
        % Solves for a cubic (3rd order) polynomial trajectory between two via-points.
        % Takes in desired starting and ending times t0 and tf (in seconds), starting
        % and ending positions (q0 and qf), and starting and ending velocities (v0 and vf).
        % Outputs a 4-by-1 array containing the coefficients ai (i = 0, 1, 2 ,3) of
        % the polynomial.
        function A = cubic_traj(t0, tf, q0, qf, v0, vf)
            M = [1, t0, t0 ^ 2, t0 ^ 3;
                 0, 1, 2 * t0, 3 * t0 ^ 2;
                 1, tf, tf ^ 2, tf ^ 3;
                 0, 1, 2 * tf, 3 * tf ^ 2];
            A = M \ [q0; v0; qf; vf];
        end

        % Solves for a quintic polynomial trajectory (5th order) between two via-points.
        % Takes in desired starting and ending times t0 and tf (in seconds), starting
        % and ending positions (q0 and qf), starting and ending velocities (v0 and vf),
        % and starting and ending accelerations (a0 and af).
        % Outputs a 6-by-1 array containing the coefficients ai (i = 0, 1, 2, 3, 4, 5) of
        % the polynomial.
        function A = quintic_traj(t0, tf, q0, qf, v0, vf, a0, af)
            M = [1, t0, t0 ^ 2, t0 ^ 3, t0 ^ 4, t0 ^ 5;
                 0, 1, 2 * t0, 3 * t0 ^ 2, 4 * t0 ^ 3, 5 * t0 ^ 4;
                 0, 0, 2, 6 * t0, 12 * t0 ^ 2, 20 * t0 ^ 3;
                 1, tf, tf ^ 2, tf ^ 3, tf ^ 4, tf ^ 5;
                 0, 1, 2 * tf, 3 * tf ^ 2, 4 * tf ^ 3, 5 * tf ^ 4;
                 0, 0, 2, 6 * tf, 12 * tf ^ 2, 20 * tf ^ 3];
            A = M \ [q0; v0; a0; qf; vf; af];
        end

        % Creates a cubic trajectory between an initial via-point and a final via-point
        % The via-points can either be in joint space or task space
        function trajectory = createCubicTrajectory(initial, final, duration)
            trajectory = zeros(4);

            for i = 1:4
                trajectory(:, i) = Traj_Planner.cubic_traj(0, duration, initial(1, i), final(1, i), 0, 0);
            end

        end

        % Creates a quintic trajectory between an initial via-point and a final via-point
        % The via-points can either be in joint space or task space
        function trajectory = createQuinticTrajectory(initial, final, duration)
            trajectory = zeros(6, 4);

            for i = 1:4
                trajectory(:, i) = Traj_Planner.quintic_traj(0, duration, initial(1, i), final(1, i), 0, 0, 0, 0);
            end

        end

    end

end
