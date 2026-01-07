# Quick Start Guide - Assignment 2

This guide helps you quickly get started with Assignment 2, following the same structure as Assignment 1.

## Structure Overview

```
assignment2/
├── README.md          ← Start here! Simple usage guide
├── report.md          ← Full report with images (like Assignment 1)
├── frames/            ← Images directory (like Assignment 1)
├── *.m files          ← 9 MATLAB files (your code)
└── Supporting docs    ← Test scripts and guides
```

## Quick Steps

### 1. Read the README
Open [README.md](README.md) for a quick overview and usage instructions.

### 2. Run the Tests
```matlab
cd('D:/Masters/Robotics/assignment2')
test_assignment2
```

This will verify all 15 points worth of functionality.

### 3. Generate Images/Frames
```matlab
capture_frames
```

This automatically generates all images for the report (saved in `frames/` directory).

### 4. View the Report
Open [report.md](report.md) to see the comprehensive documentation with all images embedded (similar to Assignment 1 format).

## File Guide

### Must-Read Files
1. **README.md** - Usage instructions and file listing
2. **report.md** - Complete report with problem, approach, images, and results

### Code Files (9 total)
**Basic (6 files):**
- RotX.m, RotY.m, RotZ.m - Rotation matrices
- Trans3D.m - Translation matrix
- plot_frame_3d.m - Plot 3D frames
- plot_robot_3d.m - Plot 3-link robot

**Advanced (3 files):**
- animate_robot_3d.m - Animate robot
- plot_robot_flexible.m - N-link robot on any axis
- arbitrary_rotation.m - Any rotation sequence (XYZ, ZYX, etc.)

### Helper Files
- **test_assignment2.m** - Run all tests
- **example_usage.m** - Code examples
- **capture_frames.m** - Generate images
- **SCREENSHOT_GUIDE.md** - Manual image capture instructions

## Quick Test Examples

### Test Rotation Matrices
```matlab
RotX(pi/4)   % Rotate 45° about X
RotY(pi/3)   % Rotate 60° about Y
RotZ(pi/6)   % Rotate 30° about Z
```

### Test Translation
```matlab
Trans3D(1, 2, 3)  % Translate (1,2,3)
```

### Plot 3D Frame
```matlab
figure;
plot_frame_3d(eye(4), 1, 'Base');
```

### Plot Robot
```matlab
figure;
plot_robot_3d(0, pi/4, pi/6);
```

### Animate Robot
```matlab
animate_robot_3d(1, 1, 1, 50);  % 50 frames (faster)
```

### Flexible Robot
```matlab
link_params = {'Z', 1.0; 'Y', 1.5; 'Y', 1.0; 'X', 0.8};
joint_angles = [pi/4, pi/6, pi/3, pi/4];
figure;
plot_robot_flexible(link_params, joint_angles);
```

### Arbitrary Rotations
```matlab
T = arbitrary_rotation('XYZ', [pi/4, pi/6, pi/3]);
```

## Submission Checklist

- [ ] All code files tested with `test_assignment2`
- [ ] Images generated with `capture_frames`
- [ ] README.md reviewed
- [ ] report.md reviewed with embedded images
- [ ] All 15 points verified

## Points Summary

| Component | Points | Status |
|-----------|--------|--------|
| Transformation functions | 2 | ✓ |
| Frame plotting | 2 | ✓ |
| Robot plotting | 2 | ✓ |
| Animation | 3 | ✓ |
| Flexible robot | 3 | ✓ |
| Arbitrary rotations | 3 | ✓ |
| **Total** | **15** | ✓ |

## Need Help?

1. Check [README.md](README.md) for basic usage
2. Read [report.md](report.md) for detailed documentation
3. See [SCREENSHOT_GUIDE.md](SCREENSHOT_GUIDE.md) for image capture
4. Review [DIRECTORY_STRUCTURE.md](DIRECTORY_STRUCTURE.md) for organization
5. Run [example_usage.m](example_usage.m) for code examples

## Comparison with Assignment 1

This follows the exact same structure as Assignment 1:
- ✓ README.md (simple guide)
- ✓ report.md (comprehensive documentation)
- ✓ frames/ directory (images)
- ✓ Clean code organization
- ✓ Test scripts

Reference: https://github.com/abhi-mdu/RMB600_assignment1

---

**Ready to submit?** Make sure you have README.md, report.md, all .m files, and frames/ with images!
