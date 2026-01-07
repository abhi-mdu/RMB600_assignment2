function T = Trans3D(x, y, z)
    % Trans3D - Creates a 4x4 homogeneous transformation matrix for translation
    %
    % Inputs:
    %   x - translation along x-axis
    %   y - translation along y-axis
    %   z - translation along z-axis
    %
    % Outputs:
    %   T - 4x4 homogeneous transformation matrix
    
    T = [1,  0,  0,  x;
         0,  1,  0,  y;
         0,  0,  1,  z;
         0,  0,  0,  1];
end
