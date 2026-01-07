function T = RotZ(angle)
    % RotZ - Creates a 4x4 homogeneous transformation matrix for rotation about Z-axis
    %
    % Inputs:
    %   angle - rotation angle in radians
    %
    % Outputs:
    %   T - 4x4 homogeneous transformation matrix
    
    c = cos(angle);
    s = sin(angle);
    
    T = [c,  -s,  0,  0;
         s,   c,  0,  0;
         0,   0,  1,  0;
         0,   0,  0,  1];
end
