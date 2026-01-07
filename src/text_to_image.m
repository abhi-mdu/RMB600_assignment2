% text_to_image.m
% Convert text output files to PNG images

% Read transformation matrices output
fid = fopen('frames/trans_matrices_output.txt', 'r');
content = fread(fid, '*char')';
fclose(fid);

% Create figure with text
figure('Position', [100, 100, 800, 600], 'Color', 'w');
axis off;
text(0.05, 0.95, content, 'FontName', 'Courier', 'FontSize', 10, ...
     'VerticalAlignment', 'top', 'Interpreter', 'none');
print('frames/trans_matrices.png', '-dpng', '-r150');
close;
fprintf('Created: trans_matrices.png\n');

% Read arbitrary rotation output
fid = fopen('frames/arbitrary_rotation_output.txt', 'r');
content = fread(fid, '*char')';
fclose(fid);

% Create figure with text
figure('Position', [100, 100, 800, 800], 'Color', 'w');
axis off;
text(0.05, 0.95, content, 'FontName', 'Courier', 'FontSize', 9, ...
     'VerticalAlignment', 'top', 'Interpreter', 'none');
print('frames/arbitrary_rotation.png', '-dpng', '-r150');
close;
fprintf('Created: arbitrary_rotation.png\n');

% Read test output
fid = fopen('frames/test_output.txt', 'r');
content = fread(fid, '*char')';
fclose(fid);

% Create figure with text
figure('Position', [100, 100, 900, 1200], 'Color', 'w');
axis off;
text(0.05, 0.95, content, 'FontName', 'Courier', 'FontSize', 8, ...
     'VerticalAlignment', 'top', 'Interpreter', 'none');
print('frames/test_output.png', '-dpng', '-r150');
close;
fprintf('Created: test_output.png\n');

fprintf('\nAll text files converted to images!\n');
