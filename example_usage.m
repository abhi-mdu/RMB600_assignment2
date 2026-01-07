% example_usage.m
% Example usage scenarios for Assignment 2

clear all;
close all;
clc;

fprintf('=== Assignment 2: Example Usage Scenarios ===\n\n');

%% Example 1: Basic Transformations
fprintf('Example 1: Creating and combining transformations\n');
fprintf('-----------------------------------------------\n');

% Create a composite transformation
T = Trans3D(1, 0, 0) * RotZ(pi/4) * Trans3D(0, 0, 1) * RotY(pi/6);
fprintf('Composite transformation created\n\n');

%% Example 2: Visualizing Multiple Frames
fprintf('Example 2: Visualizing multiple coordinate frames\n');
fprintf('--------------------------------------------------\n');

figure('Name', 'Example 2: Multiple Frames', 'Position', [100, 100, 800, 600]);
hold on;

% Base frame
T0 = eye(4);
plot_frame_3d(T0, 1, 'World');

% Frame 1: Translate and rotate
T1 = Trans3D(2, 0, 0) * RotZ(pi/4);
plot_frame_3d(T1, 0.8, 'Frame 1');

% Frame 2: From Frame 1
T2 = T1 * Trans3D(1.5, 0, 0) * RotY(pi/6);
plot_frame_3d(T2, 0.8, 'Frame 2');

% Frame 3: From Frame 2
T3 = T2 * Trans3D(1, 0, 0) * RotX(pi/4);
plot_frame_3d(T3, 0.8, 'Frame 3');

title('Chain of Coordinate Frames');
hold off;
fprintf('Multiple frames plotted\n\n');

pause(2);

%% Example 3: Simple Robot in Different Poses
fprintf('Example 3: Robot in different configurations\n');
fprintf('--------------------------------------------\n');

% Pose 1: Home position
figure('Name', 'Example 3a: Home Position');
plot_robot_3d([0, 0, 0]);

pause(1);

% Pose 2: Extended
figure('Name', 'Example 3b: Extended');
plot_robot_3d([pi/4, pi/4, pi/4]);

pause(1);

% Pose 3: Folded
figure('Name', 'Example 3c: Folded');
plot_robot_3d([pi/2, -pi/3, pi/3]);

fprintf('Three robot poses shown\n\n');

pause(2);

%% Example 4: Flexible Robot - SCARA-like configuration
fprintf('Example 4: SCARA-like robot configuration\n');
fprintf('-----------------------------------------\n');

link_params_scara = {
    'Z', 1.0;   % Vertical base
    'Z', 1.5;   % First arm (rotate in XY plane)
    'Z', 1.2;   % Second arm (rotate in XY plane)
    'Z', 0.3    % Tool
};

joint_angles_scara = [pi/6, pi/4, -pi/3, 0];
figure('Name', 'Example 4: SCARA Configuration');
plot_robot_flexible(link_params_scara, joint_angles_scara);
title('SCARA-like Robot Configuration');

fprintf('SCARA robot plotted\n\n');

pause(2);

%% Example 5: Flexible Robot - Articulated arm
fprintf('Example 5: Articulated arm configuration\n');
fprintf('----------------------------------------\n');

link_params_arm = {
    'Z', 0.5;   % Base
    'Y', 1.0;   % Shoulder
    'Y', 1.0;   % Elbow
    'Y', 0.5;   % Wrist
    'X', 0.3    % End-effector
};

joint_angles_arm = [pi/4, pi/6, pi/3, pi/4, pi/6];
figure('Name', 'Example 5: Articulated Arm');
plot_robot_flexible(link_params_arm, joint_angles_arm);
title('Articulated Arm Configuration');

fprintf('Articulated arm plotted\n\n');

pause(2);

%% Example 6: Comparing Rotation Sequences
fprintf('Example 6: Comparing different rotation sequences\n');
fprintf('-------------------------------------------------\n');

angles = [pi/4, pi/6, pi/3];

fprintf('\nSame angles, different sequences:\n');
fprintf('Angles: [%.2f, %.2f, %.2f] radians\n', angles);
fprintf('       [%.1f, %.1f, %.1f] degrees\n\n', rad2deg(angles));

T_xyz = arbitrary_rotation('XYZ', angles);
T_zyx = arbitrary_rotation('ZYX', angles);
T_xyx = arbitrary_rotation('XYX', angles);

% Visualize the resulting orientations
figure('Name', 'Example 6: Rotation Sequences', 'Position', [100, 100, 1200, 400]);

subplot(1, 3, 1);
hold on;
plot_frame_3d(eye(4), 0.5, 'Base');
plot_frame_3d(T_xyz, 1, 'XYZ');
title('XYZ Rotation');
view(45, 30);
hold off;

subplot(1, 3, 2);
hold on;
plot_frame_3d(eye(4), 0.5, 'Base');
plot_frame_3d(T_zyx, 1, 'ZYX');
title('ZYX Rotation');
view(45, 30);
hold off;

subplot(1, 3, 3);
hold on;
plot_frame_3d(eye(4), 0.5, 'Base');
plot_frame_3d(T_xyx, 1, 'XYX');
title('XYX Rotation');
view(45, 30);
hold off;

fprintf('Rotation sequences compared\n\n');

pause(2);

%% Example 7: Robot Reaching Different Points
fprintf('Example 7: Robot reaching different target points\n');
fprintf('-------------------------------------------------\n');

% Different configurations to reach different points
configs = [
    0,     0,     0;       % Home
    pi/4,  pi/6,  pi/6;   % Point 1
    pi/2,  pi/4,  pi/4;   % Point 2
    -pi/4, -pi/6, pi/3;   % Point 3
];

for i = 1:size(configs, 1)
    figure('Name', sprintf('Example 7: Configuration %d', i));
    plot_robot_3d(configs(i, :));
    title(sprintf('Configuration %d: [%.2f, %.2f, %.2f]', ...
          i, configs(i, 1), configs(i, 2), configs(i, 3)));
    pause(1);
end

fprintf('Robot configurations shown\n\n');

%% Summary
fprintf('\n=== Examples Completed ===\n');
fprintf('All example scenarios have been demonstrated.\n');
fprintf('You can modify these examples or create your own!\n');
