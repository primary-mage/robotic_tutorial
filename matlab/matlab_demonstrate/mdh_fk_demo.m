function mdh_fk_demo()
%MDH_FK_DEMO Modified-DH forward kinematics demo (with tool frame).
%   Input joint angles q (rad), print link transforms A_i, wrist pose T_04,
%   and tool pose T_tool, then plot the robot using the Robotics Toolbox.
%
%   Difference from mdh_teach_demo.m: a tool frame is added,
%       T_tool = TransX(0.07) * RotY(pi/2)
%   i.e. frame4 translated 0.07 m along X, then rotated 90 deg about Y.
%
%   Usage: type mdh_fk_demo in the command window, then enter q.

    % ---- 1. Build robot (modified DH, same params as mdh_teach_demo.m) ----
    L(1) = Link([0 0.12 0.00 0.00], 'modified');
    L(2) = Link([0 0.00 0.00 pi/2], 'modified');
    L(3) = Link([0 0.00 0.18 0.00], 'modified');
    L(4) = Link([0 0.00 0.18 0.00], 'modified');

    robot = SerialLink(L, 'name', 'MDH Demo');

    % ---- 2. Tool frame: frame4 +0.07 m along X, then +90 deg about Y ----
    robot.tool = transl(0.07, 0, 0) * troty(pi/2);

    % ---- 3. Input joint angles ----
    fprintf('Enter joint angles q (rad). Press Enter for example [0.4 1.2 0.7 -0.5]:\n');
    q = str2num(input('q = ', 's')); %#ok<ST2NM>
    if isempty(q)
        q = [0.4 1.2 0.7 -0.5];
    end

    % ---- 4. Forward kinematics ----
    fprintf('\n================ Link transforms ================\n');
    T04 = eye(4);
    for i = 1:robot.n
        Ai = double(robot.links(i).A(q(i)));
        T04 = T04 * Ai;
        fprintf('\nA%d =\n', i);
        disp(Ai);
    end

    Ttool = double(robot.fkine(q));   % tool pose (includes robot.tool)
    pos = Ttool(1:3, 4);              % TCP position
    ztool = Ttool(1:3, 3);            % approach direction (Z_tool)
    az = atan2(ztool(2), ztool(1));   % approach azimuth  (= q1)
    el = atan2(ztool(3), hypot(ztool(1), ztool(2)));  % approach elevation (= q2+q3+q4)

    fprintf('\n================ Forward kinematics ================\n');
    fprintf('joint angles q = [%.6f %.6f %.6f %.6f] rad\n', q);
    fprintf('\nT04 (wrist) =\n');
    disp(T04);
    fprintf('\nT_tool =\n');
    disp(Ttool);
    fprintf('\nTCP position      = [%.6f %.6f %.6f] m\n', pos);
    fprintf('Approach dir (Zt) = [%.6f %.6f %.6f]\n', ztool);
    fprintf('Approach azimuth  = %.6f rad  (= q1)\n', az);
    fprintf('Approach elev.    = %.6f rad  (= q2+q3+q4)\n', el);
    fprintf('==================================================\n');

    % ---- 5. Plot ----
    robot.plot(q, ...
        'workspace', [-0.4 0.5 -0.4 0.5 -0.1 0.6], ...
        'floorlevel', 0, ...
        'jointdiam', 1.2);
    title('MDH forward kinematics (with tool frame)');
end
