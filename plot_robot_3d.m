function plot_robot_3d(joint_angles)
    % plot_robot_3d - Plots a 3D robot with at least three links
    %
    % Inputs:
    %   joint_angles - vector of joint angles in radians [theta1, theta2, theta3, ...]
    %                  Default: [0, 0, 0]
    
    if nargin < 1
        joint_angles = [0, 0, 0];
    end
    
    % Ensure we have at least 3 joints
    if length(joint_angles) < 3
        joint_angles(end+1:3) = 0;
    end
    
    % Robot parameters (link lengths)
    L1 = 1;  % First link length
    L2 = 1;  % Second link length
    L3 = 1;  % Third link length
    
    % Base frame (world frame)
    T0 = eye(4);
    
    % Joint 1 transformation (rotation about Z-axis)
    T1 = T0 * RotZ(joint_angles(1)) * Trans3D(0, 0, L1);
    
    % Joint 2 transformation (rotation about Y-axis)
    T2 = T1 * RotY(joint_angles(2)) * Trans3D(L2, 0, 0);
    
    % Joint 3 transformation (rotation about Y-axis)
    T3 = T2 * RotY(joint_angles(3)) * Trans3D(L3, 0, 0);
    
    % Clear figure and create new plot
    figure;
    hold on;
    
    % Plot coordinate frames
    plot_frame_3d(T0, 0.3, 'Base');
    plot_frame_3d(T1, 0.3, 'Joint1');
    plot_frame_3d(T2, 0.3, 'Joint2');
    plot_frame_3d(T3, 0.3, 'End-Effector');
    
    % Extract link positions
    p0 = T0(1:3, 4);
    p1 = T1(1:3, 4);
    p2 = T2(1:3, 4);
    p3 = T3(1:3, 4);
    
    % Plot links as lines
    plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], ...
          'k-', 'LineWidth', 4);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
          'k-', 'LineWidth', 4);
    plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], ...
          'k-', 'LineWidth', 4);
    
    % Plot joints as spheres
    plot3(p0(1), p0(2), p0(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(p1(1), p1(2), p1(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot3(p2(1), p2(2), p2(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(p3(1), p3(2), p3(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Set plot properties
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Robot with 3 Links');
    view(3);
    
    % Set axis limits
    max_reach = L1 + L2 + L3;
    xlim([-max_reach, max_reach]);
    ylim([-max_reach, max_reach]);
    zlim([0, max_reach]);
    
    hold off;
end
