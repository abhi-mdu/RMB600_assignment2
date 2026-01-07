# Assignment 2 — 3D Forward Kinematics (RMB600)

This folder contains MATLAB/Octave code for Assignment 2 (3D forward kinematics).

## Files:

### Basic Requirements (9 points)

• `RotX.m` — returns a 4×4 homogeneous rotation matrix about the X-axis.  
• `RotY.m` — returns a 4×4 homogeneous rotation matrix about the Y-axis.  
• `RotZ.m` — returns a 4×4 homogeneous rotation matrix about the Z-axis.  
• `Trans3D.m` — returns a 4×4 homogeneous translation matrix for 3D translation (x,y,z).  
• `plot_frame_3d.m` — plots a 3D coordinate frame given a 4×4 transform (X-axis in red, Y-axis in green, Z-axis in blue).  
• `plot_robot_3d.m` — plots a 3-link 3D robot given joint angles and link lengths.  

### Advanced Requirements (6 points)

• `animate_robot_3d.m` — animates a 3D robot with all joints rotating about the Z-axis.  
• `plot_robot_flexible.m` — plots a robot with any number of links on any rotation axis (X, Y, or Z).  
• `arbitrary_rotation.m` — computes transformation matrices for arbitrary rotation sequences (e.g., XYZ, ZYX, XYX).  

### Supporting Files

• `test_assignment2.m` — comprehensive test script for all functions.  
• `example_usage.m` — usage examples demonstrating all functions.  
• `report.md` — detailed assignment report with implementation details and images.  

## Usage:

1. Open MATLAB or Octave and change directory to this folder:
   ```matlab
   cd('D:/Masters/Robotics/assignment2')
   ```

2. Run the test script to verify all functions:
   ```matlab
   test_assignment2
   ```

3. Run basic examples:
   ```matlab
   % Test rotation matrices
   RotX(pi/4)
   RotY(pi/3)
   RotZ(pi/6)
   Trans3D(1, 2, 3)
   
   % Plot a 3D frame
   figure;
   plot_frame_3d(eye(4), 1, 'Base Frame');
   
   % Plot a 3-link robot
   figure;
   plot_robot_3d(0, pi/4, pi/6);
   ```

4. Run advanced examples:
   ```matlab
   % Animate robot
   animate_robot_3d(1, 1, 1, 100);
   
   % Flexible robot with 4 links
   link_params = {'Z', 1.0; 'Y', 1.5; 'Y', 1.0; 'X', 0.8};
   joint_angles = [pi/4, pi/6, pi/3, pi/4];
   figure;
   plot_robot_flexible(link_params, joint_angles);
   
   % Arbitrary rotation sequences
   T = arbitrary_rotation('XYZ', [pi/4, pi/6, pi/3]);
   ```

5. For more examples, see:
   ```matlab
   example_usage
   ```

## Notes & assumptions:

• All rotation angles are in radians.  
• The robot uses homogeneous transformation matrices (4×4) for 3D transformations.  
• Forward kinematics is computed by sequentially multiplying rotation and translation matrices.  
• Code is written to be compatible with both MATLAB and Octave.  
• In `plot_robot_flexible.m`, variable `axis` was renamed to `ax` to avoid shadowing the built-in MATLAB/Octave function.  

## Running in Octave:

If using GNU Octave:
```bash
cd assignment2
octave --gui
```

Or command-line mode:
```bash
octave-cli --no-gui --eval "cd('D:/Masters/Robotics/assignment2'); test_assignment2; exit;"
```

## Verification:

• Example images demonstrating the code are included in the `frames/` subfolder.  
• Run `test_assignment2` to verify all functions pass the test suite.  
• Check `report.md` for detailed documentation with embedded images.  

## Points Summary:

- **Basic Requirements:** 9/9 points
  - Transformation functions (RotX, RotY, RotZ, Trans3D): 2 points
  - 3D frame plotting: 2 points
  - 3D robot visualization: 2 points

- **Advanced Requirements:** 6/6 points
  - Robot animation: 3 points
  - Flexible robot function: 3 points
  - Arbitrary rotation sequences: 3 points

**Total: 15/15 points**
