# RMB600 Assignment 2 - 3D Forward Kinematics

MATLAB/Octave implementation of 3D robot forward kinematics with visualization and animation capabilities.

## 📁 Directory Structure

```
├── src/                          # MATLAB source code
│   ├── RotX.m                    # Rotation about X-axis
│   ├── RotY.m                    # Rotation about Y-axis
│   ├── RotZ.m                    # Rotation about Z-axis
│   ├── Trans3D.m                 # 3D translation
│   ├── plot_frame_3d.m           # Plot coordinate frames
│   ├── plot_robot_3d.m           # Plot 3-link robot
│   ├── plot_robot_flexible.m     # Plot flexible robot configuration
│   ├── animate_robot_3d.m        # Animate robot motion
│   ├── arbitrary_rotation.m      # Arbitrary rotation sequences
│   ├── test_assignment2.m        # Test suite
│   ├── example_usage.m           # Usage examples
│   └── generate_report_figures.m # Generate all report figures
│
├── docs/                         # Documentation
│   ├── README.md                 # Detailed project documentation
│   ├── ASSIGNMENT2_SUBMISSION_REPORT.md  # Submission report
│   ├── QUICK_START.md            # Quick start guide
│   ├── report.md                 # Detailed technical report
│   └── *.pdf, *.html             # Generated reports
│
├── matlab_assignment2/           # Generated figures (from MATLAB Online)
└── frames/                       # Example output frames

```

## 🚀 Quick Start

### Running Locally (MATLAB/Octave)

```matlab
cd src
test_assignment2        % Run all tests
example_usage          % See usage examples
```

### Running on MATLAB Online

1. Clone the repository:
   ```matlab
   !git clone https://github.com/abhi-mdu/RMB600_assignment2.git
   cd RMB600_assignment2/src
   ```

2. Generate figures:
   ```matlab
   generate_report_figures
   ```

3. Run tests:
   ```matlab
   test_assignment2
   ```

## 📊 Features

### Basic Requirements (9 points)
- ✅ Transformation matrices (RotX, RotY, RotZ, Trans3D)
- ✅ 3D coordinate frame visualization
- ✅ 3-link robot plotting

### Advanced Requirements (6 points)
- ✅ Robot animation with rotating joints
- ✅ Flexible robot configuration (any number of links, any rotation axes)
- ✅ Arbitrary rotation sequences (XYZ, ZYX, XYX, etc.)

## 📖 Documentation

- [Quick Start Guide](docs/QUICK_START.md)
- [Submission Report](docs/ASSIGNMENT2_SUBMISSION_REPORT.md)
- [Technical Report](docs/report.md)
- [Deliverables Checklist](docs/DELIVERABLES.md)

## 🎓 Course Information

- **Course:** RMB600 - Robot Modelling
- **Assignment:** Assignment Two: 3D Forward Kinematics
- **Institution:** Högskolan Väst, Trollhättan, Sweden

## 📝 Usage Examples

```matlab
% Basic transformations
T = RotZ(pi/4) * Trans3D(1, 0, 0);

% Plot a frame
plot_frame_3d(T, 1.0);

% Plot a 3-link robot
joint_angles = [pi/4, pi/3, pi/6];
link_lengths = [1, 1.5, 1];
plot_robot_3d(joint_angles, link_lengths);

% Flexible robot with mixed axes
joint_angles = [pi/6, pi/4, -pi/6, pi/3];
link_lengths = [1, 1.2, 0.8, 1];
rotation_axes = ['Z', 'Y', 'Z', 'X'];
plot_robot_flexible(joint_angles, link_lengths, rotation_axes);
```

## 📦 Requirements

- MATLAB R2019b or later / GNU Octave 6.0+
- No additional toolboxes required

## ✅ Test Results

All tests pass successfully: **15/15 points**

```
✓ Basic Requirements (9/9 points)
✓ Advanced Requirements (6/6 points)
```

## 📄 License

Educational project for RMB600 course.

## 👤 Author

Abhishek Kumar  
Masters in AI, Högskolan Väst
