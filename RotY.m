function T = RotY(angle)
    % RotY - Creates a 4x4 homogeneous transformation matrix for rotation about Y-axis
    %
    % Inputs:
    %   angle - rotation angle in radians
    %
    % Outputs:
    %   T - 4x4 homogeneous transformation matrix
    
    c = cos(angle);
    s = sin(angle);
    
    T = [c,   0,  s,  0;
         0,   1,  0,  0;
        -s,   0,  c,  0;
         0,   0,  0,  1];
end
