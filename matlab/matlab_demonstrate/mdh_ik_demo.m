function mdh_ik_demo()
%MDH_IK_DEMO Modified-DH inverse kinematics demo (with tool frame).
%   Input the TCP in Cartesian (x, y, z) OR cylindrical (rho, theta, z)
%   coordinates plus the approach elevation phi (rad). Solve with the
%   closed-form geometric method, print candidate solutions, and plot the
%   robot using the Robotics Toolbox.
%
%   Difference from mdh_teach_demo.m: a tool frame is added,
%       T_tool = TransX(0.07) * RotY(pi/2)
%   The IK first backs the TCP out to the wrist position, then solves the
%   planar 2R subproblem (law of cosines).
%
%   Cylindrical coordinates are natural for this arm: theta IS q1, and the
%   radial distance rho plus height z define the planar 2R subproblem.
%
%   Usage: type mdh_ik_demo in the command window, then follow the prompts.

    % ---- 1. Build robot (same as mdh_teach_demo.m) ----
    L(1) = Link([0 0.12 0.00 0.00], 'modified');
    L(2) = Link([0 0.00 0.00 pi/2], 'modified');
    L(3) = Link([0 0.00 0.18 0.00], 'modified');
    L(4) = Link([0 0.00 0.18 0.00], 'modified');

    robot = SerialLink(L, 'name', 'MDH Demo');

    % ---- 2. Tool frame: frame4 +0.07 m along X, then +90 deg about Y ----
    robot.tool = transl(0.07, 0, 0) * troty(pi/2);

    % ---- 3. Joint limits (same as URDF) ----
    qlim = [-pi pi; 0 pi; -0.8*pi 0.8*pi; -2*pi/3 2*pi/3];

    % ---- 4. Input target (Cartesian or cylindrical) ----
    fprintf('Choose coordinate system for the TCP position:\n');
    fprintf('  1 = Cartesian   (x, y, z)\n');
    fprintf('  2 = Cylindrical (rho, theta, z)   <-- theta is the azimuth q1\n');
    mode = input('Select 1 or 2 (Enter for 1): ');
    if isempty(mode)
        mode = 1;
    end

    if mode == 2
        fprintf('Enter [rho theta z]. Press Enter for example [0.2973 0.3430 0.30]:\n');
        p = str2num(input('TCP (rho theta z) = ', 's')); %#ok<ST2NM>
        if isempty(p)
            p = [0.2973 0.3430 0.30];
        end
        rho_c = p(1); theta_c = p(2); z = p(3);
        x = rho_c * cos(theta_c);
        y = rho_c * sin(theta_c);
    else
        fprintf('Enter [x y z]. Press Enter for example [0.28 0.10 0.30]:\n');
        p = str2num(input('TCP (x y z) = ', 's')); %#ok<ST2NM>
        if isempty(p)
            p = [0.28 0.10 0.30];
        end
        x = p(1); y = p(2); z = p(3);
        rho_c = hypot(x, y);
        theta_c = atan2(y, x);
    end

    phi = input('Enter approach elevation phi (rad). Press Enter for example 0: ');
    if isempty(phi)
        phi = 0;
    end

    % ---- 5. Closed-form geometric IK ----
    a = 0.18;
    q1 = theta_c;                      % azimuth: q1 = theta directly
    rho = rho_c - 0.07 * cos(phi);     % tool offset: TCP -> wrist
    zp = z - 0.12 - 0.07 * sin(phi);

    sols = {};
    configs = {};

    if rho < 0
        fprintf('\nUnreachable: wrist would fall behind the base.\n');
    else
        D = hypot(rho, zp);
        if D > 2 * a
            fprintf('\nUnreachable: D=%.4f exceeds workspace limit 0.36.\n', D);
        else
            c3 = (D^2 - 2*a^2) / (2*a^2);
            c3 = max(-1, min(1, c3));
            for s = [1 -1]               % elbow-down / elbow-up branches
                q3 = s * acos(c3);
                q2 = atan2(zp, rho) - atan2(a*sin(q3), a + a*cos(q3));
                q4 = phi - q2 - q3;
                q = [q1 q2 q3 q4];
                if all(q >= qlim(:,1)' & q <= qlim(:,2)')
                    sols{end+1} = q; %#ok<AGROW>
                    if s > 0
                        configs{end+1} = 'elbow-down'; %#ok<AGROW>
                    else
                        configs{end+1} = 'elbow-up'; %#ok<AGROW>
                    end
                end
            end
        end
    end

    % ---- 6. Print and verify ----
    if isempty(sols)
        fprintf('No solution within joint limits.\n');
        return;
    end

    fprintf('\n================ Inverse kinematics ================\n');
    fprintf('Target TCP  (x,y,z)    = [%.6f %.6f %.6f] m\n', x, y, z);
    fprintf('Target TCP  (rho,th,z) = [%.6f %.6f %.6f]\n', rho_c, theta_c, z);
    fprintf('Approach elevation phi = %.6f rad\n', phi);
    for i = 1:numel(sols)
        q = sols{i};
        T = double(robot.fkine(q));
        pos = T(1:3, 4);
        zt = T(1:3, 3);
        el = atan2(zt(3), hypot(zt(1), zt(2)));
        err = norm(pos - [x y z]');
        fprintf('[%s] q = [%.6f %.6f %.6f %.6f] rad\n', configs{i}, q);
        fprintf('     FK check TCP = [%.6f %.6f %.6f], pos err = %.2e\n', pos, err);
        fprintf('     approach elev = %.6f rad (target %.6f)\n', el, phi);
    end
    fprintf('===================================================\n');

    % ---- 7. Plot all solutions overlaid ----
    W = [-0.4 0.5 -0.4 0.5 -0.1 0.6];
    robot.plot(sols{1}, 'workspace', W, 'floorlevel', 0, 'jointdiam', 1.2);
    hold on;
    for i = 2:numel(sols)
        robot.plot(sols{i}, 'workspace', W, 'floorlevel', 0, 'jointdiam', 1.2);
    end
    hold off;
    title('MDH inverse kinematics (multiple configs to same TCP)');
end
