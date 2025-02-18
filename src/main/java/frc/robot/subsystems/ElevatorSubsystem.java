// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

  // Motor controller for the elevator

  private final TalonFX elevatorMotor1 = new TalonFX(Elevator.LEFT_MOTOR_ID, "rio");
  private final TalonFX elevatorMotor2 = new TalonFX(Elevator.RIGHT_MOTOR_ID, "rio");

  SoftwareLimitSwitchConfigs limitSwitchConfigs = new SoftwareLimitSwitchConfigs()
  .withForwardSoftLimitEnable(true)
  .withForwardSoftLimitThreshold(Elevator.MAX_SAFE_POS)
  .withReverseSoftLimitEnable(true)
  .withReverseSoftLimitThreshold(0);

  
  

  // PID controller for the elevator
  private final ProfiledPIDController pidController;
  private double pidOutput;
  private double feedforward;

  private boolean PIDEnabled = false;

  private final ElevatorFeedforward m_ElevatorFeedforward =
      new ElevatorFeedforward(Elevator.KS, Elevator.KG, Elevator.KV, Elevator.KA);

  // Elevator setpoint (desired position)
  private double setpoint;

  public ElevatorSubsystem() {
    // Initialize motor controller

    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    // Configure encoder (adjust these values based on your setup)

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    fx_cfg.Slot0.kV = Elevator.MOTOR_KV;
    fx_cfg.Slot0.kP = Elevator.MOTOR_KP;
    fx_cfg.Slot0.kI = Elevator.MOTOR_KI;
    fx_cfg.Slot0.kD = Elevator.MOTOR_KD;
    fx_cfg.Slot0.kS = Elevator.MOTOR_KS;

    fx_cfg.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    //limit switch
    fx_cfg.withSoftwareLimitSwitch(limitSwitchConfigs);  
    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor1.getConfigurator().apply(fx_cfg);

    // TODO: Position Conversion Factor: Setting it to 1.0 leaves encoder units as motor rotations.
    // Convert to real-world units (e.g., meters) using gear ratios or pulley dimensions

    // motorConfig.encoder.positionConversionFactor(1.0);

    // elevatorMotor.configure(
    //     motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize PID controller
    pidController =
        new ProfiledPIDController(
            Elevator.KP,
            Elevator.KI,
            Elevator.KD,
            new TrapezoidProfile.Constraints(Elevator.MAX_VELOCITY, Elevator.MAX_ACCELERATION));

    pidController.setTolerance(
        Elevator.PID_POSITION_TOLERANCE, Elevator.PID_VELOCITY_TOLERANCE); // Set the
    // tolerance to
    // 0.1 units
    // (e.g., inches)

    // Set the initial setpoint to the current position
    setpoint = getEncoderPosition();
  }

  // Get the current encoder position
  public double getEncoderPosition() {
    return elevatorMotor1.getRotorPosition().getValueAsDouble();
  }

  // Set the desired setpoint (elevator position)
  public void setSetpoint(double setpoint) {
    this.setpoint = Math.max(0, Math.min(Elevator.MAX_SAFE_POS, setpoint));
    pidController.setGoal(this.setpoint);
  }

  // Function to check if PID method is complete
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  // Run the PID controller to move the elevator to the setpoint
  public void runPID() {
    // Calculate the PID output
    this.pidOutput = pidController.calculate(getEncoderPosition());

    // TODO: Implement motion profiling (e.g., TrapezoidProfile) to dynamically compute
    // velocity/acceleration setpoints.
    this.feedforward = m_ElevatorFeedforward.calculate(pidController.getSetpoint().velocity);

    // Set the motor output
    setClampSpeed(pidOutput + feedforward);
  }

  // Move the elevator manually (for manual driving)
  public void moveManual(double speed) {
    // Disable PID control and set the motor speed directly
    this.disablePID();
    // if(getEncoderPosition() < Elevator.MAX_SAFE_POS && getEncoderPosition() > 0){
    setClampSpeed(speed);
  //   } else{
  //     setClampSpeed(0);
  //   }
  //    if(getEncoderPosition() == 0){
  //     if(speed > 0){
  //       setClampSpeed(speed);
  //     }
  //    }
  }

  private void setClampSpeed(double speed) {
    elevatorMotor1.set((Math.max(-1, Math.min(1, speed))));
  }

  // Stop the elevator motor
  public void stop() {
    this.disablePID();
  }

  public void enablePID() {
    pidController.reset(getEncoderPosition());
    PIDEnabled = true;
  }

  public void disablePID() {
    PIDEnabled = false;
    setSetpoint(getEncoderPosition()); // Reset setpoint to current position
    setClampSpeed(0);
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  public boolean isStalled() {
    // Check stall condition - Stall velocity is less than defined constant & stallCurrentThreshold is exceeded
    double current = elevatorMotor1.getRotorVelocity().getValueAsDouble();
    double velocity = elevatorMotor1.getSupplyCurrent().getValueAsDouble();
    return Math.abs(velocity) < Elevator.STALL_VELOCITY_THRESHOLD && current > Elevator.STALL_CURRENT_THRESHOLD;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (PIDEnabled == true) {
      runPID();
    }

    // Update SmartDashboard
    // Log outputs using AdvantageKit
    Logger.recordOutput("Elevator/Setpoint", setpoint);
    Logger.recordOutput("Elevator/Position", getEncoderPosition());
    Logger.recordOutput("Elevator/Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Elevator/PIDEnabled", PIDEnabled);
    Logger.recordOutput("Elevator/AtSetpoint", atSetpoint());
    Logger.recordOutput("Elevator/PIDOutput", pidOutput);
    Logger.recordOutput("Elevator/Feedforward", feedforward);
    Logger.recordOutput("Elevator/TotalOutput", pidOutput + feedforward);
  }
}
