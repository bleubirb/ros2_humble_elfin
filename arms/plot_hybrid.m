
TOLERANCE = 0.05;
DELTA2 = 0.3;
DELTA1 = 0.1;

% Data
data = {
    "No Hysteresis", 41, 50, false; 
    "No Hysteresis", 43, 52, false; 
    "No Hysteresis", 44, 53, false; 
    % "Stress Ball, 5N", 59, 68, 213, 218, true; % pos = 0.040, 0.620, 0.4; rot = -90, -112, 0;    "Stress Ball, 5N", 60, 69, 219, 224, true; 
    % "Stress Ball, 7N", 61, 70, 225, 230, true; 
    % "Stress Ball, 3N", 62, 71, 231, 236, true; 
    % "Stress Ball, 3N, No Hysteresis", 63, 72, 237, 242, false; 
    % "Stress Ball, 5N, No Hysteresis", 64, 73, 243, 248, false; 
    % "Stress Ball, 7N, No Hysteresis", 65, 74, 249, 254, false; 
    % "Stress Ball, Dropped", 71, 81, 286, 291, true; 
    % "Ball 1", 1027, 1029, NaN, NaN, true;
    % "Ball 2", 1034, 1033, NaN, NaN, true;
    % "Test 9", 1008, 1010, NaN, NaN, true;
    % "Test 13", 1017, 1019, NaN, NaN, true;
    % "Ball", 89, 101, NaN, NaN, true;
    % "Ball, 3N to 1N, Hysteresis", 106, 144, true;
    % "Ball, 3N to 1N, Hysteresis", 112, 159, true;
    % "Temperature", 94, 115, true;
    % "Squeeze", 145, 168, true;
    % "Fill", 146, 169, true;
    % "Rotation", 134, 156, true;
    % "Long Baseline", 154, 177, false;
    % "Fill Baseline", 156, 179, false;
    % "Squeeze Baseline", 157, 180, false;
    % "Rotation Baseline", 159, 183, false;
};

% Iterate over data
for idx = 1:size(data, 1)
    title = data{idx, 1};
    gripper = data{idx, 2};
    state = data{idx, 3};
    % Support both legacy 6-col rows [title, gripper, state, arm_first, arm_end, hyst]
    % and newer 4-col rows [title, gripper, state, hyst]
    numCols = size(data, 2);
    if numCols >= 6
        arm_first = data{idx, 4};
        arm_end = data{idx, 5};
        hyst = data{idx, 6};
    elseif numCols >= 4
        arm_first = [];
        arm_end = [];
        hyst = data{idx, 4};
    else
        arm_first = [];
        arm_end = [];
        hyst = false;
    end
    
    fprintf('%s\n', title);
    start_time = inf;

    % Gripper file
    g_file = [];
    if ~isempty(gripper)
        g_file_path = sprintf('../arms/data/hybrid_gripper_%d.csv', gripper);
        if isfile(g_file_path)
            g_file = readmatrix(g_file_path);
            start_time = min(start_time, g_file(2, 1));
        else
            fprintf("Couldn't find %s\n", g_file_path);
        end
    end

    % State file
    s_file = [];
    if ~isempty(state)
        s_file_path = sprintf('../arms/data/hybrid_state_%d.csv', state);
        if isfile(s_file_path)
            s_file = readmatrix(s_file_path);
            start_time = min(start_time, s_file(2, 1));
        else
            fprintf("Couldn't find %s\n", s_file_path);
        end
    end

    figure;

    % Gripper state over time
    if ~isempty(g_file)
        time_2 = [0.0];
        gripper_state_2 = [0];
        for i = 2:size(g_file, 1)
            t = g_file(i, 1) - start_time;
            time_2(end+1) = t;
            % Map raw gripper state (col 5) to {-1,0,1}
            if g_file(i, 5) == 0
                gripper_state_2(end+1) = 0;
            elseif g_file(i, 5) > 0
                gripper_state_2(end+1) = 1;
            else
                gripper_state_2(end+1) = -1;
            end
        end

        subplot(2, 1, 1); % First subplot
    plot(time_2, gripper_state_2, 'LineWidth', 2);
        set(gcf, "Theme", "Light");
        set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
        ylim([-2, 2]);
        % xlim([0, 35]);
        xlabel('Time (s)');
        ylabel('State');
    yticks([-1, 0, 1]);
    yticklabels({'L', 'H', 'T'});
        xlim([25, 90]);
        % title(sprintf('%s - Gripper State', title));
    end

    % Gripper force over time
    if ~isempty(g_file)
        time_3 = g_file(:, 1) - start_time;
        force_3 = g_file(:, 3);
        fd_3 = g_file(:, 4);
        fd_3(abs(diff([0; fd_3])) > 0) = NaN;

        valid_indices = time_3 >= 9 & time_3 <= 1800;
        time_3 = time_3(valid_indices);
        force_3 = force_3(valid_indices);
        fd_3 = fd_3(valid_indices);

        subplot(2, 1, 2); % Second subplot
    plot(time_3, force_3, 'k', 'LineWidth', 1.5);
        % fontname(gcf, "Courier 10 Pitch");
        set(gcf, "Theme", "Light");
        set(gca, 'FontName', 'Times New Roman', 'FontSize', 14);
        hold on;
        if hyst
            % Constant ±DELTA hysteresis band around desired force
            fd_top = fd_3 + DELTA2;
            fd_bot = fd_3 - DELTA2;
            fd_d1 = fd_3 + DELTA1;
            fd_d2 = fd_3 - DELTA1;
            plot(time_3, fd_bot, '--r', 'LineWidth', 2, 'DisplayName', 'Hysteresis Lower Bound');
            plot(time_3, fd_top, '--r', 'LineWidth', 2, 'DisplayName', 'Hysteresis Upper Bound');
            plot(time_3, fd_d1, ':b', 'LineWidth', 2, 'DisplayName', 'Inner Bound 1');
            plot(time_3, fd_d2, ':b', 'LineWidth', 2, 'DisplayName', 'Inner Bound 2');
        end
        xlabel('Time (s)');
        ylabel('Force (N)');
        ylim([0, 4]);
        xlim([25, 90]);
        % title(sprintf('%s - Gripper Force', title));
    end

    % Save the combined figure
    saveas(gcf, sprintf('plots/%d_%s_combined.png', idx, title));
    % close;
end

