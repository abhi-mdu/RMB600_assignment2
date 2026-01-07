function T = RotX(angle)
    % RotX - Creates a 4x4 homogeneous transformation matrix for rotation about X-axis
    %
    % Inputs:
    %   angle - rotation angle in radians
    %
    % Outputs:
    %   T - 4x4 homogeneous transformation matrix
    
    c = cos(angle);
    s = sin(angle);
    
    T = [1,  0,  0,  0;
         0,  c, -s,  0;
         0,  s,  c,  0;
         0,  0,  0,  1];
end
