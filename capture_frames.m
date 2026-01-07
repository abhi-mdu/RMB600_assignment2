% capture_frames.m
% Automated script to capture all frames for Assignment 2 report
% Similar to Assignment 1 structure

fprintf('=== Capturing frames for Assignment 2 ===\n\n');

% Create frames directory if it doesn't exist
if ~exist('frames', 'dir')
    mkdir('frames');
    fprintf('Created frames/ directory\n');
end

%% 1. Transformation Matrices Output
fprintf('1. Capturing transformation matrices output...\n');
diary('frames/trans_matrices_output.txt');
fprintf('\n=== RotX(pi/4) ===\n');
RotX(pi/4)
fprintf('\n=== RotY(pi/3) ===\n');
RotY(pi/3)
fprintf('\n=== RotZ(pi/6) ===\n');
RotZ(pi/6)
fprintf('\n=== Trans3D(1,2,3) ===\n');
Trans3D(1,2,3)
diary off;
fprintf('   Saved text output to trans_matrices_output.txt\n');

%% 2. 3D Frame Plotting
fprintf('2. Capturing 3D frame plotting...\n');
figure('Position', [100, 100, 800, 600]);
T1 = eye(4);
T2 = RotZ(pi/4) * Trans3D(1, 0, 0);
T3 = RotY(pi/3) * Trans3D(0, 1, 1);
plot_frame_3d(T1, 0.5, 'Base');
hold on;
plot_frame_3d(T2, 0.5, 'Frame A');
plot_frame_3d(T3, 0.5, 'Frame B');
hold off;
grid on;
title('3D Coordinate Frames Visualization');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
print('frames/plot_frame_3d.png', '-dpng', '-r150');
fprintf('   Saved: plot_frame_3d.png\n');
close;

%% 3. 3D Robot Plotting
fprintf('3. Capturing 3D robot plot...\n');
figure('Position', [100, 100, 800, 600]);
plot_robot_3d([0, pi/4, pi/6]);
view(45, 30);
print('frames/plot_robot_3d.png', '-dpng', '-r150');
fprintf('   Saved: plot_robot_3d.png\n');
close;

%% 4. Robot Animation - Capture middle frame
fprintf('4. Capturing animation frame...\n');
figure('Position', [100, 100, 800, 600]);
% Capture frame 50 out of 100
L1 = 1; L2 = 1; L3 = 1;
theta1 = 2*pi * 50/100;
theta2 = pi * 50/100;
theta3 = pi/2 * 50/100;

T0 = eye(4);
T1 = T0 * RotZ(theta1) * Trans3D(0, 0, L1);
T2 = T1 * RotZ(theta2) * Trans3D(L2, 0, 0);
T3 = T2 * RotZ(theta3) * Trans3D(L3, 0, 0);

% Plot links
origins = [T0(1:3,4), T1(1:3,4), T2(1:3,4), T3(1:3,4)];
plot3(origins(1,:), origins(2,:), origins(3,:), 'k-', 'LineWidth', 3);
hold on;

% Plot joints
plot3(T0(1,4), T0(2,4), T0(3,4), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
plot3(T1(1,4), T1(2,4), T1(3,4), 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
plot3(T2(1,4), T2(2,4), T2(3,4), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
plot3(T3(1,4), T3(2,4), T3(3,4), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

% Plot frames
plot_frame_3d(T0, 0.3, 'Base');
plot_frame_3d(T1, 0.3, 'J1');
plot_frame_3d(T2, 0.3, 'J2');
plot_frame_3d(T3, 0.3, 'EE');

grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Animation - Frame 50/100');
view(45, 30);
axis equal;
hold off;
print('frames/animate_frame_050.png', '-dpng', '-r150');
fprintf('   Saved: animate_frame_050.png\n');
close;

%% 5. Flexible Robot
fprintf('5. Capturing flexible robot...\n');
figure('Position', [100, 100, 800, 600]);
link_params = {'Z', 1.0; 'Y', 1.5; 'Y', 1.0; 'X', 0.8};
joint_angles = [pi/4, pi/6, pi/3, pi/4];
plot_robot_flexible(link_params, joint_angles);
view(45, 30);
print('frames/plot_robot_flexible.png', '-dpng', '-r150');
fprintf('   Saved: plot_robot_flexible.png\n');
close;

%% 6. Arbitrary Rotation Sequences
fprintf('6. Capturing arbitrary rotation output...\n');
diary('frames/arbitrary_rotation_output.txt');
fprintf('\n=== Test 1: XYZ Rotation ===\n');
T1 = arbitrary_rotation('XYZ', [pi/4, pi/6, pi/3]);
fprintf('\n=== Test 2: ZYX Rotation ===\n');
T2 = arbitrary_rotation('ZYX', [pi/3, pi/6, pi/4]);
fprintf('\n=== Test 3: XYX Rotation ===\n');
T3 = arbitrary_rotation('XYX', [pi/4, pi/3, pi/4]);
fprintf('\n=== Test 4: YZY Rotation ===\n');
T4 = arbitrary_rotation('YZY', [pi/6, pi/4, pi/3]);
diary off;
fprintf('   Saved text output to arbitrary_rotation_output.txt\n');

%% 7. Test Output
fprintf('7. Running test suite and capturing output...\n');
diary('frames/test_output.txt');
test_assignment2
diary off;
fprintf('   Saved: test_output.txt\n');

fprintf('\n=== Frame capture complete! ===\n');
fprintf('All frames saved to frames/ directory\n');
fprintf('\nFrames captured:\n');
fprintf('  - plot_frame_3d.png\n');
fprintf('  - plot_robot_3d.png\n');
fprintf('  - animate_frame_050.png\n');
fprintf('  - plot_robot_flexible.png\n');
fprintf('  - trans_matrices_output.txt (convert to PNG as needed)\n');
fprintf('  - arbitrary_rotation_output.txt (convert to PNG as needed)\n');
fprintf('  - test_output.txt (convert to PNG as needed)\n');
fprintf('\nFor text outputs, you can screenshot the terminal or convert to PNG.\n');
