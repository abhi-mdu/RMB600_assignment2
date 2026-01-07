function T = arbitrary_rotation(sequence, angles)
    % arbitrary_rotation - Creates transformation matrix for arbitrary rotation sequence
    %
    % Inputs:
    %   sequence - string specifying rotation order (e.g., 'XYZ', 'ZYX', 'XYX')
    %   angles - vector of rotation angles in radians [angle1, angle2, angle3]
    %
    % Outputs:
    %   T - 4x4 homogeneous transformation matrix
    %
    % Example usage:
    %   T = arbitrary_rotation('XYZ', [pi/4, pi/6, pi/3]);
    %   T = arbitrary_rotation('ZYX', [0.5, 0.3, 0.1]);
    
    % Validate inputs
    if nargin < 2
        error('Both sequence and angles are required');
    end
    
    if length(sequence) ~= length(angles)
        error('Number of angles must match sequence length');
    end
    
    % Convert sequence to uppercase
    sequence = upper(sequence);
    
    % Validate sequence contains only X, Y, Z
    valid_chars = {'X', 'Y', 'Z'};
    for i = 1:length(sequence)
        if ~ismember(sequence(i), valid_chars)
            error('Sequence must only contain X, Y, or Z');
        end
    end
    
    % Initialize with identity matrix
    T = eye(4);
    
    % Apply rotations in sequence
    for i = 1:length(sequence)
        axis = sequence(i);
        angle = angles(i);
        
        switch axis
            case 'X'
                T = T * RotX(angle);
            case 'Y'
                T = T * RotY(angle);
            case 'Z'
                T = T * RotZ(angle);
        end
    end
    
    % Display rotation sequence and resulting matrix
    fprintf('Rotation Sequence: %s\n', sequence);
    fprintf('Angles (rad): [');
    fprintf('%.4f ', angles);
    fprintf(']\n');
    fprintf('Angles (deg): [');
    fprintf('%.2f ', rad2deg(angles));
    fprintf(']\n\n');
    fprintf('Resulting Transformation Matrix:\n');
    disp(T);
end
