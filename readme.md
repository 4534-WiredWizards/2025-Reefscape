# Reefscape 2025 Robot Code Repository


<!-- Table Of Contents -->
## 📚 Table Of Contents
- [🚧 Current Work](#-current-work)
- [🛠️ Highlevel Goals](#️-highlevel-goals)
  - [Drivetrain](#drivetrain)
  - [Path Planning](#path-planning)
  - [Vision Integration](#vision-integration)
- [💫 Resources](#-resources
  - [Elevator](#elevator)
  - [Vision](#vision)
- [🤖 Repository Structure](#-repository-structure)
    - [Branch Strategy](#branch-strategy)
    - [Branch Management](#branch-management)


## 🚧 Current Work





## 🛠️ Highlevel Goals

### Elevator
- [x] Subsystem
- [x] Manual and PID control
- [ ] Tune PID and feedforward
> Future Work
 - [ ] Position Conversion Factor: Setting it to 1.0 leaves encoder units as motor rotations. Convert to real-world units (e.g., meters) using gear ratios or pulley dimensions
- [ ] Implement motion profiling (e.g., TrapezoidProfile) to dynamically compute velocity/acceleration setpoints `m_ElevatorFeedforward.calculate(C_Elevator.TargetVelocity)`


### Drivetrain
- [x] Import Advantage Kit drive code
- [ ] Complete full diagnostic tuning for swerve

### Path Planning
- [x] Evaluate **Path Planner**
- [ ] Integrate path planning solution

### Vision Integration
- [x] Decision: Use Limelight (preferred from previous year)
- [ ] Implement Limelight setup from Advantage Kit base code
- [ ] Configure forward and rear-facing camera



## 💫 Resources
- [Advantage Kit Vision Integration Docs](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#vision-integration)
- [Path Planner Documentation](https://pathplanner.dev/home.html)
- [Limelight Documentation](https://docs.limelightvision.io/en/latest/)

### Elevator:
- [Using Feedforward Components with PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html?utm_source=chatgpt.com#using-feedforward-components-with-pid)
- [SysId, PID, and Feedforward Tuning for Elevator](https://www.chiefdelphi.com/t/sysid-pid-and-feedfoward-tuning-for-elevator/482797)
- [Elevator Relative Speed](https://www.chiefdelphi.com/t/elevator-relative-speed/482861)
### Vision:
- [How to Check Which April Tag the Limelight is Seeing](https://www.chiefdelphi.com/t/how-to-check-which-april-tag-the-limelight-is-seeing/483990/2)


## 🤖 Repository Structure

### Branch Strategy
Base branch structure:
- `Gillbert`: Swerve base code
- `develop`: Active development and integration
- `main`: Stable, compitition-ready code 🤖🥳

### Branch Management
1. Set up branch protection rules
2. Team development occurs in `develop` branch
3. Max reviews all Pull Requests
4. Merge from `develop` to `main` only after thorough testing
5. Limit base branch modifications to Max and Greg
