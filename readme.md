# 🐠 Team 4534 Robot Code Repository

**Next-gen FIRST robotics codebase for efficient reef scoring automation 😎**

---

## 📚 Table of Contents
- [🎯 High-Level Goals](#-high-level-goals)
- [🚀 Features & Progress](#-features--progress)
  - [Elevator](#elevator)
  - [Wrist](#wrist)
  - [Drivetrain](#drivetrain)
  - [Path Planning](#path-planning)
  - [Vision Integration](#vision-integration)
- [📖 Resources](#-resources)
  - [Guides](#guides)
  - [Subsystems](#subsystems)
- [🌳 Repository Structure](#-repository-structure)
  - [Branch Strategy](#branch-strategy)
  - [Workflow](#workflow)

---

## 🎯 High-Level Goals

- **Drivetrain**: Optimize swerve responsiveness + odometry accuracy.
- **Path Planning**: Seamless autonomous navigation via Path Planner.
- **Vision**: Real-time AprilTag tracking + multi-camera support.

---

## 🚀 Features & Progress

### **Elevator**

- Tune PID + feedforward
- Real-world position conversion (meters via pulley ratios)
- Motion profiling (`TrapezoidProfile`)
- Safety: Software limits + output clamping

<details>
<summary>✅ Completed Items</summary>

- Subsystem framework
- Manual + PID control commands

</details>

<details>
<summary>📁 Code Snippets</summary>

```java
// Position conversion  
elevatorEncoder.setPositionConversionFactor(C_Elevator.ENCODER_DISTANCE_PER_PULSE);  
elevatorEncoder.setVelocityConversionFactor(C_Elevator.ENCODER_VELOCITY_FACTOR);

// Clamping motor output 
double output = pidOutput + feedforward;  
output = Math.max(-1, Math.min(1, output));  
elevatorMotor.set(output); 
```

</details>

---

### **Wrist**

- Subsystem setup
- Manual + PID controls
- Motion profiling

<details>
<summary>✅ Completed Items</summary>

- None yet

</details>

---

### **Drivetrain**

- Full swerve diagnostic tuning
- Odometry calibration
- Path following optimization

<details>
<summary>✅ Completed Items</summary>

- Swerve base code (Advantage Kit)

</details>

---

### **Path Planning**

- Integration with codebase
- Auto routine builder
- Constraint tuning

<details>
<summary>✅ Completed Items</summary>

- Path Planner evaluation

</details>

---

### **Vision Integration**

- Advantage Kit + Limelight setup
- Dual camera configuration
- AprilTag pose estimation

<details>
<summary>✅ Completed Items</summary>

- Limelight selected (legacy compatibility)

</details>

---

## 📖 Resources

### **Guides**
- [Advantage Kit Vision Docs](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/#vision-integration)
- [Path Planner Tutorials](https://pathplanner.dev/home.html)
- [Limelight Setup Guide](https://docs.limelightvision.io/en/latest/)
- [REV Spark Flex API](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkflex)

### **Subsystems**

- **Elevator**
    - 🔗 [PID + Feedforward Tuning](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html)
    - 🔗 [SysId Best Practices](https://www.chiefdelphi.com/t/sysid-pid-and-feedfoward-tuning-for-elevator/482797)

- **Vision**
  - 🔗 [AprilTag Filtering](https://www.chiefdelphi.com/t/how-to-check-which-april-tag-the-limelight-is-seeing/483990/2)

---

## 🌳 Repository Structure

### **Branch Strategy**
- `Gillbert`: Swerve prototype base 🧪
- `develop`: Active integration + testing 🔄
- `main`: Competition-ready stable releases 🏆

### **Workflow**
1. **Protect Branches**: Require PR reviews for `main`/`develop`.
2. **Develop in `develop`**: All features tested here first.
3. **PR Reviews**: Approved by leads (Max/Greg) before merging.
4. **Release to `main`**: Only after rigorous validation.

---

**Made with ❤️ by Team 4534 - The Wired Wizards**