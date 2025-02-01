// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.C_Wrist;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private final SparkFlex wristMotor;

  private final SparkFlex rollerMotor;

  private DutyCycleEncoder absEncoder;

  private final PIDController pidController;

  private boolean PIDEnabled = false;

  private final ArmFeedforward m_WristFeedforward =
      new ArmFeedforward(C_Wrist.kS, C_Wrist.kG, C_Wrist.kV, C_Wrist.kA);

  private double setpoint;

  public WristSubsystem() {
    pidController = new PIDController(C_Wrist.kP, C_Wrist.kI, C_Wrist.kD);
    pidController.setTolerance(C_Wrist.PIDPositionTolerance, C_Wrist.PIDVelocityTolerance);

    wristMotor = new SparkFlex(C_Wrist.pivotMotorID, SparkLowLevel.MotorType.kBrushless);
    rollerMotor = new SparkFlex(C_Wrist.Roller.MotorID, SparkLowLevel.MotorType.kBrushless);

    absEncoder =
        new DutyCycleEncoder(
            C_Wrist.Encoder.port, C_Wrist.Encoder.fullRange, C_Wrist.Encoder.expectedZero);

    SparkFlexConfig brakeConfig = new SparkFlexConfig();
    SparkFlexConfig IdleConfig = new SparkFlexConfig();

    brakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    IdleConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);

    wristMotor.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(
        IdleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set initial setpoint
    setpoint = getAngle();
  }

  public double getAngle() {
    return ((((-1 * absEncoder.get()) + C_Wrist.AbsEncoderOffset + 1) % 1.0) * (2 * Math.PI));
  }

  public void setWristSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void runPID() {
    double pidoutput = pidController.calculate(getAngle(), setpoint);
    wristMotor.set(
        pidoutput + m_WristFeedforward.calculate(Units.degreesToRadians(getAngle()), pidoutput));
    double feedforward =
        m_WristFeedforward.calculate(Units.degreesToRadians(getAngle()), C_Wrist.TargetVelocity);
    wristMotor.set(pidoutput + feedforward);
  }

  public void moveManual(double speed) {
    this.disable();
    wristMotor.set(speed);
  }

  public void moveRoller(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRoller() {
    rollerMotor.set(0);
  }

  public void stop() {
    this.disable();
  }

  public void enable() {
    PIDEnabled = true;
  }

  public void disable() {
    PIDEnabled = false;
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  public void moveToPosition(double position) {
    // TODO
  }

  @Override
  public void periodic() {
    if (PIDEnabled == true) {
      runPID();
    }
  }
}
