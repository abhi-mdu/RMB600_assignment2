function animate_robot_3d()
    % animate_robot_3d - Animates a 3D robot moving its joints on Z-axis direction
    %
    % This script creates an animation of a robot moving through different
    % joint configurations with rotations about the Z-axis
    
    % Robot parameters
    L1 = 1;  % First link length
    L2 = 1;  % Second link length  
    L3 = 1;  % Third link length
    
    % Animation parameters
    n_frames = 100;
    theta1_range = linspace(0, 2*pi, n_frames);
    theta2_range = linspace(0, pi, n_frames);
    theta3_range = linspace(0, pi/2, n_frames);
    
    % Create figure
    figure('Position', [100, 100, 800, 600]);
    
    % Animation loop
    for i = 1:n_frames
        clf;  % Clear figure
        hold on;
        
        % Current joint angles
        theta1 = theta1_range(i);
        theta2 = theta2_range(i);
        theta3 = theta3_range(i);
        
        % Base frame
        T0 = eye(4);
        
        % Joint transformations (all rotating about Z-axis)
        T1 = T0 * RotZ(theta1) * Trans3D(0, 0, L1);
        T2 = T1 * RotZ(theta2) * Trans3D(0, 0, L2);
        T3 = T2 * RotZ(theta3) * Trans3D(0, 0, L3);
        
        % Plot coordinate frames
        plot_frame_3d(T0, 0.3, 'Base');
        plot_frame_3d(T1, 0.3, 'J1');
        plot_frame_3d(T2, 0.3, 'J2');
        plot_frame_3d(T3, 0.3, 'EE');
        
        % Extract positions
        p0 = T0(1:3, 4);
        p1 = T1(1:3, 4);
        p2 = T2(1:3, 4);
        p3 = T3(1:3, 4);
        
        % Plot links
        plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], ...
              'k-', 'LineWidth', 4);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'k-', 'LineWidth', 4);
        plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], ...
              'k-', 'LineWidth', 4);
        
        % Plot joints
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
        title(sprintf('3D Robot Animation (Frame %d/%d)', i, n_frames));
        view(45, 30);
        
        % Set axis limits
        max_reach = L1 + L2 + L3;
        xlim([-max_reach, max_reach]);
        ylim([-max_reach, max_reach]);
        zlim([0, max_reach]);
        
        hold off;
        
        % Pause for animation
        pause(0.05);
        drawnow;
    end
    
    disp('Animation completed!');
end
