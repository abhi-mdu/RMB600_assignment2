function plot_frame_3d(T, scale, label_text)
    % plot_frame_3d - Plots a 3D coordinate frame
    %
    % Inputs:
    %   T - 4x4 homogeneous transformation matrix
    %   scale - length of axis arrows (default: 1)
    %   label_text - text label for the frame (default: '')
    
    if nargin < 2
        scale = 1;
    end
    if nargin < 3
        label_text = '';
    end
    
    % Extract origin position
    origin = T(1:3, 4);
    
    % Extract rotation matrix (orientation axes)
    x_axis = T(1:3, 1) * scale;
    y_axis = T(1:3, 2) * scale;
    z_axis = T(1:3, 3) * scale;
    
    % Hold current plot
    hold on;
    
    % Plot X-axis (red)
    quiver3(origin(1), origin(2), origin(3), ...
            x_axis(1), x_axis(2), x_axis(3), ...
            'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Plot Y-axis (green)
    quiver3(origin(1), origin(2), origin(3), ...
            y_axis(1), y_axis(2), y_axis(3), ...
            'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Plot Z-axis (blue)
    quiver3(origin(1), origin(2), origin(3), ...
            z_axis(1), z_axis(2), z_axis(3), ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % Add label if provided
    if ~isempty(label_text)
        text(origin(1), origin(2), origin(3), ['  ' label_text], ...
             'FontSize', 10, 'FontWeight', 'bold');
    end
    
    % Set axis properties
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);
    
    hold off;
end
