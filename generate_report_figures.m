% generate_report_figures.m
% Generates all figures for the assignment report
% Run this in MATLAB Online to create updated figures

fprintf('Generating figures for Assignment 2 report...\n\n');

% Create matlab_assignment2 directory if it doesn't exist
if ~exist('matlab_assignment2', 'dir')
    mkdir('matlab_assignment2');
end

%% 1. Basic Transformation Matrices
fprintf('1. Generating transformation matrices example...\n');
figure('Position', [100, 100, 800, 600]);
clf;

% Test rotations
theta = pi/4;
Rx = RotX(theta);
Ry = RotY(theta);
Rz = RotZ(theta);
T = Trans3D(1, 2, 3);

% Plot coordinate frames
subplot(2,2,1);
plot_frame_3d(eye(4), 0.5);
hold on;
plot_frame_3d(Rx, 0.5);
title('Rotation about X-axis (45°)');
view(3);
grid on;
axis equal;
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);

subplot(2,2,2);
plot_frame_3d(eye(4), 0.5);
hold on;
plot_frame_3d(Ry, 0.5);
title('Rotation about Y-axis (45°)');
view(3);
grid on;
axis equal;
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);

subplot(2,2,3);
plot_frame_3d(eye(4), 0.5);
hold on;
plot_frame_3d(Rz, 0.5);
title('Rotation about Z-axis (45°)');
view(3);
grid on;
axis equal;
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);

subplot(2,2,4);
plot_frame_3d(eye(4), 0.5);
hold on;
plot_frame_3d(T, 0.5);
title('Translation (1, 2, 3)');
view(3);
grid on;
axis equal;

saveas(gcf, 'matlab_assignment2/trans_matrices.png');
fprintf('   Saved: matlab_assignment2/trans_matrices.png\n');

%% 2. 3D Frame Plotting
fprintf('2. Generating 3D frame plot example...\n');
figure('Position', [100, 100, 800, 600]);
clf;

T_combined = Trans3D(2, 1, 1) * RotZ(pi/6) * RotY(pi/4);
plot_frame_3d(eye(4), 1);
hold on;
plot_frame_3d(T_combined, 1);
title('3D Coordinate Frame Visualization');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
grid on;
axis equal;
legend('Base Frame', 'Transformed Frame', 'Location', 'best');

saveas(gcf, 'matlab_assignment2/plot_frame_3d.png');
fprintf('   Saved: matlab_assignment2/plot_frame_3d.png\n');

%% 3. 3D Robot with 3 Links
fprintf('3. Generating 3D robot plot...\n');
figure('Position', [100, 100, 800, 600]);
clf;

joint_angles = [pi/4, pi/3, pi/6];
link_lengths = [1, 1.5, 1];
plot_robot_3d(joint_angles, link_lengths);
title('3-Link Robot (3D Forward Kinematics)');
view(45, 30);

saveas(gcf, 'matlab_assignment2/plot_robot_3d.png');
fprintf('   Saved: matlab_assignment2/plot_robot_3d.png\n');

%% 4. Flexible Robot Configuration
fprintf('4. Generating flexible robot example...\n');
figure('Position', [100, 100, 800, 600]);
clf;

joint_angles = [pi/6, pi/4, -pi/6, pi/3];
link_lengths = [1, 1.2, 0.8, 1];
rotation_axes = ['Z', 'Y', 'Z', 'X'];
end_pos = plot_robot_flexible(joint_angles, link_lengths, rotation_axes);
title('Flexible Robot with Mixed Rotation Axes');
view(45, 30);

saveas(gcf, 'matlab_assignment2/plot_robot_flexible.png');
fprintf('   Saved: matlab_assignment2/plot_robot_flexible.png\n');
fprintf('   End-effector position: [%.3f, %.3f, %.3f]\n', end_pos);

%% 5. Arbitrary Rotation Sequence
fprintf('5. Generating arbitrary rotation example...\n');
figure('Position', [100, 100, 800, 600]);
clf;

angles = [pi/6, pi/4, pi/3];
sequence = 'XYZ';
[T_final, T_list] = arbitrary_rotation(angles, sequence);

% Plot each transformation step
plot_frame_3d(eye(4), 0.5);
hold on;
colors = {'r', 'g', 'b', 'k'};
for i = 1:length(T_list)
    plot_frame_3d(T_list{i}, 0.5);
end
title(sprintf('Arbitrary Rotation Sequence: %s', sequence));
view(3);
grid on;
axis equal;
legend('Base', 'After X', 'After XY', 'After XYZ', 'Location', 'best');

saveas(gcf, 'matlab_assignment2/arbitrary_rotation.png');
fprintf('   Saved: matlab_assignment2/arbitrary_rotation.png\n');

%% 6. Animation Frame (sample)
fprintf('6. Generating animation sample frame...\n');
figure('Position', [100, 100, 800, 600]);
clf;

% Generate a mid-animation frame
joint_angles = [pi/3, pi/4, pi/6];
link_lengths = [1, 1, 1];
plot_robot_3d(joint_angles, link_lengths);
title('Robot Animation - Sample Frame');
view(45, 30);

saveas(gcf, 'matlab_assignment2/animate_frame_050.png');
fprintf('   Saved: matlab_assignment2/animate_frame_050.png\n');

%% 7. Test Output Visualization
fprintf('7. Generating test output visualization...\n');
figure('Position', [100, 100, 1000, 600]);
clf;

% Show multiple robot configurations
configs = [
    0, 0, 0;
    pi/4, 0, 0;
    pi/4, pi/4, 0;
    pi/4, pi/4, pi/4
];

for i = 1:4
    subplot(2, 2, i);
    plot_robot_3d(configs(i,:), [1, 1, 1]);
    title(sprintf('Config %d: [%.2f, %.2f, %.2f]', i, configs(i,1), configs(i,2), configs(i,3)));
    view(45, 30);
end

saveas(gcf, 'matlab_assignment2/test_output.png');
fprintf('   Saved: matlab_assignment2/test_output.png\n');

fprintf('\nAll figures generated successfully!\n');
fprintf('Figures saved in the matlab_assignment2/ directory.\n');
fprintf('\nTo use in your report:\n');
fprintf('1. Download the PNG files from matlab_assignment2/ folder\n');
fprintf('2. Insert them into your report using markdown:\n');
fprintf('   ![Description](matlab_assignment2/filename.png)\n');
