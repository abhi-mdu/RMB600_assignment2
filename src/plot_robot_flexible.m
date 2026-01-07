function plot_robot_flexible(link_params, joint_angles)
    % plot_robot_flexible - Plots a 3D robot with any number of links on any joint
    %
    % Inputs:
    %   link_params - cell array of link parameters
    %                 Each cell contains: {axis, length}
    %                 axis: 'X', 'Y', or 'Z' for rotation axis
    %                 length: link length
    %                 Example: {'Z', 1; 'Y', 1.5; 'Y', 1; 'X', 0.5}
    %   joint_angles - vector of joint angles in radians
    %                  Must match the number of links
    %
    % Example usage:
    %   link_params = {'Z', 1; 'Y', 1.5; 'Y', 1; 'X', 0.5};
    %   joint_angles = [pi/4, pi/6, pi/3, pi/4];
    %   plot_robot_flexible(link_params, joint_angles);
    
    % Validate inputs
    if nargin < 2
        error('Both link_params and joint_angles are required');
    end
    
    n_links = size(link_params, 1);
    if length(joint_angles) ~= n_links
        error('Number of joint angles must match number of links');
    end
    
    % Initialize transformation matrices
    T = cell(n_links + 1, 1);
    T{1} = eye(4);  % Base frame
    
    % Calculate forward kinematics for each link
    for i = 1:n_links
        ax = link_params{i, 1};
        length_val = link_params{i, 2};
        angle = joint_angles(i);
        
        % Get rotation matrix based on axis
        switch upper(ax)
            case 'X'
                R = RotX(angle);
                Trans = Trans3D(length_val, 0, 0);
            case 'Y'
                R = RotY(angle);
                Trans = Trans3D(0, length_val, 0);
            case 'Z'
                R = RotZ(angle);
                Trans = Trans3D(0, 0, length_val);
            otherwise
                error('Invalid axis. Use X, Y, or Z');
        end
        
        % Compute transformation
        T{i+1} = T{i} * R * Trans;
    end
    
    % Create figure
    figure('Position', [100, 100, 900, 700]);
    hold on;
    
    % Plot all frames
    for i = 1:(n_links + 1)
        if i == 1
            label = 'Base';
        elseif i == n_links + 1
            label = 'End-Effector';
        else
            label = sprintf('J%d', i-1);
        end
        plot_frame_3d(T{i}, 0.2, label);
    end
    
    % Plot links and joints
    colors = ['k', 'r', 'g', 'b', 'm', 'c', 'y'];
    for i = 1:(n_links + 1)
        % Extract position
        p = T{i}(1:3, 4);
        
        % Plot joint
        color_idx = mod(i-1, length(colors)) + 1;
        plot3(p(1), p(2), p(3), 'o', ...
              'MarkerSize', 10, ...
              'MarkerFaceColor', colors(color_idx), ...
              'MarkerEdgeColor', 'k', ...
              'LineWidth', 1.5);
        
        % Plot link to next joint
        if i <= n_links
            p_next = T{i+1}(1:3, 4);
            plot3([p(1), p_next(1)], [p(2), p_next(2)], [p(3), p_next(3)], ...
                  'k-', 'LineWidth', 4);
        end
    end
    
    % Set plot properties
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('Flexible 3D Robot (%d links)', n_links));
    view(3);
    
    % Calculate workspace limits
    positions = zeros(n_links + 1, 3);
    for i = 1:(n_links + 1)
        positions(i, :) = T{i}(1:3, 4)';
    end
    
    max_vals = max(abs(positions));
    limit = max(max_vals) * 1.2;
    xlim([-limit, limit]);
    ylim([-limit, limit]);
    zlim([-limit, limit]);
    
    hold off;
    
    % Display end-effector position
    ee_pos = T{end}(1:3, 4);
    fprintf('End-Effector Position: [%.3f, %.3f, %.3f]\n', ...
            ee_pos(1), ee_pos(2), ee_pos(3));
end
