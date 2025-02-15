// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFX wristMotor;

  private final SparkFlex rollerMotor;

  private DutyCycleEncoder absEncoder;

  private final ProfiledPIDController pidController;

  private boolean PIDEnabled = false;

  private final ArmFeedforward m_WristFeedforward =
      new ArmFeedforward(Wrist.KS, Wrist.KG, Wrist.KV, Wrist.KA);

  private double setpoint;

  public WristSubsystem() {
    pidController =
        new ProfiledPIDController(
            Wrist.KP,
            Wrist.KI,
            Wrist.KD,
            new TrapezoidProfile.Constraints(Wrist.MAX_VELOCITY, Wrist.MAX_ACCELERATION));
    pidController.setTolerance(Wrist.PID_POSITION_TOLERANCE, Wrist.PID_VELOCITY_TOLERANCE);

    // wrist config
    var fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    wristMotor = new TalonFX(Wrist.PIVOT_MOTOR_ID);

    wristMotor.getConfigurator().apply(fx_cfg);

    absEncoder =
        new DutyCycleEncoder(
            Wrist.Encoder.PORT, Wrist.Encoder.FULL_RANGE, Wrist.Encoder.EXPECTED_ZERO);

    // intake config
    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlexConfig brakeConfig = new SparkFlexConfig();
    SparkFlexConfig IdleConfig = new SparkFlexConfig();

    brakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    IdleConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);

    rollerMotor.configure(
        IdleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set initial setpoint
    setpoint = getAngle();
  }

  public double getAngle() {
    // using motor encoder instead of absolute encoder for now
    // return ((((-1 * absEncoder.get()) + Wrist.Encoder.ABSOLUTE_OFFSET + 1) % 1.0) * (2 *
    // Math.PI));
    return ((((-1 * wristMotor.getRotorPosition().getValueAsDouble())
                + Wrist.Encoder.ABSOLUTE_OFFSET
                + 1)
            % 1.0)
        * 360);
  }

  public void setWristSetpoint(double setpoint) {
    this.setpoint = Math.max(Wrist.MIN_SAFE_ANGLE, Math.min(Wrist.MAX_SAFE_ANGLE, setpoint));
    pidController.setGoal(this.setpoint);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void runPID() {
    double pidoutput = pidController.calculate(getAngle());

    double feedforward =
        m_WristFeedforward.calculate(
            Units.degreesToRadians(getAngle()), pidController.getSetpoint().velocity);

    setClampSpeed(pidoutput + feedforward);
  }

  public void moveManual(double speed) {
    this.disable();
    setClampSpeed(speed);
  }

  private void setClampSpeed(double speed) {
    wristMotor.set((Math.max(-1, Math.min(1, speed))));
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
    pidController.reset(getAngle());
    PIDEnabled = true;
  }

  public void disable() {
    PIDEnabled = false;
    setWristSetpoint(getAngle()); // reset setpoint to current position
    setClampSpeed(0);
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
    SmartDashboard.putNumber("Wrist Setpoint", setpoint);
    SmartDashboard.putNumber("Wrist Position", getAngle());
  }
}
