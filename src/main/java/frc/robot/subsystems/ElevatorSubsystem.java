// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.C_Elevator;

public class ElevatorSubsystem extends SubsystemBase {

  // Motor controller for the elevator
  private final SparkFlex elevatorMotor;

  // Encoder for the elevator
  private final RelativeEncoder elevatorEncoder;

  // PID controller for the elevator
  private final PIDController pidController;

  private boolean PIDEnabled = false;

  private final ElevatorFeedforward m_ElevatorFeedforward =
      new ElevatorFeedforward(C_Elevator.kS, C_Elevator.kG, C_Elevator.kV, C_Elevator.kA);

  // Elevator setpoint (desired position)
  private double setpoint;

  public ElevatorSubsystem() {
    // Initialize motor controller
    elevatorMotor =
        new SparkFlex(
            C_Elevator.leftMotorID, MotorType.kBrushless); // Replace with the correct CAN ID

    // Initialize encoder
    elevatorEncoder = elevatorMotor.getEncoder();

    // Configure encoder (adjust these values based on your setup)
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    // TODO: Position Conversion Factor: Setting it to 1.0 leaves encoder units as motor rotations.
    // Convert to real-world units (e.g., meters) using gear ratios or pulley dimensions
    motorConfig.encoder.positionConversionFactor(1.0);

    elevatorMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize PID controller
    pidController = new PIDController(C_Elevator.kP, C_Elevator.kI, C_Elevator.kD);
    pidController.setTolerance(
        C_Elevator.PIDPositionTolerance, C_Elevator.PIDVelocityTolerance); // Set the
    // tolerance to
    // 0.1 units
    // (e.g., inches)

    // Set the initial setpoint to the current position
    setpoint = getEncoderPosition();
  }

  // Get the current encoder position
  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  // Set the desired setpoint (elevator position)
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  // Function to check if PID method is complete
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  // Run the PID controller to move the elevator to the setpoint
  public void runPID() {
    // Calculate the PID output
    double pidOutput = pidController.calculate(getEncoderPosition(), setpoint);

    // TODO: Implement motion profiling (e.g., TrapezoidProfile) to dynamically compute
    // velocity/acceleration setpoints.
    double feedforward = m_ElevatorFeedforward.calculate(C_Elevator.TargetVelocity);

    // Set the motor output
    elevatorMotor.set(pidOutput + feedforward);
  }

  // Move the elevator manually (for manual driving)
  public void moveManual(double speed) {
    // Disable PID control and set the motor speed directly
    this.disable();
    elevatorMotor.set(speed);
  }

  // Stop the elevator motor
  public void stop() {
    this.disable();
  }

  public void enable() {
    PIDEnabled = true;
    pidController.reset();
  }

  public void disable() {
    PIDEnabled = false;
    setSetpoint(getEncoderPosition()); // Reset setpoint to current position
    elevatorMotor.set(0);
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (PIDEnabled == true) {
      runPID();
    }
  }
}
