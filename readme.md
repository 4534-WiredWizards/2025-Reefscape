# Reefscape 2025 Robot Code Repository


<!-- Table Of Contents -->
## 📚 Table Of Contents
- [🚧 Functions & Goals](#-functions--goals)
  - [Elevator](#elevator) 
  - [Wrist](#wrist)
  - [Drivetrain](#drivetrain)
  - [Path Planning](#path-planning)
  - [Vision Integration](#vision-integration)
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


## 🚧 Functions & Goals

### Elevator
- [x] Subsystem
- [x] Manual and PID control commands
> Future Work
- [ ] Tune PID and feedforward
 - [ ] Position Conversion Factor: Setting it to 1.0 leaves encoder units as motor rotations. Convert to real-world units (e.g., meters) using gear ratios or pulley dimensions
- [ ] Implement motion profiling (e.g., TrapezoidProfile) to dynamically compute velocity/acceleration setpoints `m_ElevatorFeedforward.calculate(C_Elevator.TargetVelocity)`
- [ ] Motor Output Clamping: Summing PID and feedforward could exceed motor limits. Clamp the output:
```java
double output = pidOutput + feedforward;
output = Math.max(-1, Math.min(1, output));
elevatorMotor.set(output);
```
- [ ] Implement software limit switches to prevent elevator from exceeding bounds:
    <details>
        <summary>Show code</summary>
        <br>
        
        ```java
        public void moveManual(double speed) {
                if ((getEncoderPosition() >= C_Elevator.MAX_HEIGHT && speed > 0) ||
                        (getEncoderPosition() <= C_Elevator.MIN_HEIGHT && speed < 0)) {
                        speed = 0;
                }
                elevatorMotor.set(speed);
        }
        ```
    </details>
- [ ] Explore implementing `public SparkBaseConfig voltageCompensation(double nominalVoltage)`

### Wrist
- [ ] Subsystem
- [ ] Manual and PID control commands
> Future Work
- [ ] Tune PID and feedforward
- [ ] Implement motion profiling


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
- [REV API Robotics Spark Flex Documentation](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkflex)

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
