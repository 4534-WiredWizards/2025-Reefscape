// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final SparkFlex rollerMotor;
  private DutyCycleEncoder absEncoder;

  private final ProfiledPIDController pidController;
  private boolean PIDEnabled = false;

  private final ArmFeedforward m_WristFeedforward =
      new ArmFeedforward(Wrist.KS, Wrist.KG, Wrist.KV, Wrist.KA);

  private double setpoint;
  private double pidOutput = 0;
  private double feedforward = 0;

  public WristSubsystem() {
    pidController =
        new ProfiledPIDController(
            Wrist.KP,
            Wrist.KI,
            Wrist.KD,
            new TrapezoidProfile.Constraints(Wrist.MAX_VELOCITY, Wrist.MAX_ACCELERATION));
    pidController.setTolerance(Wrist.PID_POSITION_TOLERANCE, Wrist.PID_VELOCITY_TOLERANCE);

    // Wrist config
    var fx_cfg = new TalonFXConfiguration();

    // Software limit switch configuration
    SoftwareLimitSwitchConfigs limitSwitches =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold((Wrist.MAX_SAFE_ANGLE * Wrist.GEAR_RATIO))
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold((Wrist.MIN_SAFE_ANGLE * Wrist.GEAR_RATIO));

    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fx_cfg.withSoftwareLimitSwitch(limitSwitches);
    fx_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    wristMotor = new TalonFX(Wrist.PIVOT_MOTOR_ID);
    wristMotor.getConfigurator().apply(fx_cfg);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

    absEncoder = new DutyCycleEncoder(Wrist.Encoder.PORT);

    // Intake config
    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlexConfig idleConfig = new SparkFlexConfig();
    idleConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    rollerMotor.configure(
        idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setpoint = getAngle();
  }

  public double getAngle() {
    double motorRotations = wristMotor.getRotorPosition().getValueAsDouble();
    double wristRotations = motorRotations * Wrist.GEAR_RATIO; // Adjust for gear ratio 45:1
    return (((-1 * wristRotations + Wrist.Encoder.ABSOLUTE_OFFSET + 1) % 1.0) * 360);
  }

  public void setWristSetpoint(double setpoint) {
    this.setpoint = Math.max(Wrist.MIN_SAFE_ANGLE, Math.min(Wrist.MAX_SAFE_ANGLE, setpoint));
    pidController.setGoal(this.setpoint);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void runPID() {
    this.pidOutput = pidController.calculate(getAngle());
    this.feedforward =
        m_WristFeedforward.calculate(
            Units.degreesToRadians(getAngle()), pidController.getSetpoint().velocity);
    setClampSpeed(pidOutput + feedforward);
  }

  public void moveManual(double speed) {
    this.disablePID();
    setClampSpeed(speed);
  }

  private void setClampSpeed(double speed) {

    double newSpeed = Math.max(-1, Math.min(1, speed));
    Logger.recordOutput("SimpleMoveWrist/Clamp", "Clamp Speed: " + newSpeed);
    wristMotor.set(newSpeed);
  }

  public void moveRoller(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRoller() {
    rollerMotor.set(0);
  }

  public void stop() {
    this.disablePID();
  }

  public boolean isStalled() {
    // Check stall condition - Stall velocity is less than defined constant & stallCurrentThreshold
    // is exceeded
    double current = wristMotor.getRotorVelocity().getValueAsDouble();
    double velocity = wristMotor.getRotorVelocity().getValueAsDouble();
    return Math.abs(velocity) < Wrist.STALL_VELOCITY_THRESHOLD
        && Math.abs(current) > Wrist.STALL_CURRENT_THRESHOLD;
  }

  public void enablePID() {
    pidController.reset(getAngle());
    PIDEnabled = true;
  }

  public void disablePID() {
    PIDEnabled = false;
    setWristSetpoint(getAngle());
    setClampSpeed(0);
    pidOutput = 0;
    feedforward = 0;
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  @Override
  public void periodic() {
    if (PIDEnabled) {
      runPID();
    }

    // Log outputs using AdvantageKit
    Logger.recordOutput("Wrist/Setpoint", setpoint);
    Logger.recordOutput("Wrist/CurrentAngle", getAngle());
    Logger.recordOutput("Wrist/RawEncoderValue", wristMotor.getRotorPosition().getValueAsDouble());
    Logger.recordOutput("Wrist/AbsoluteEncoderValue", absEncoder.get());
    Logger.recordOutput("Wrist/PIDOutput", pidOutput);
    Logger.recordOutput("Wrist/FeedforwardOutput", feedforward);
    Logger.recordOutput("Wrist/TotalMotorOutput", pidOutput + feedforward);
    Logger.recordOutput("Wrist/MotorVoltage", wristMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Wrist/PIDEnabled", PIDEnabled);
    Logger.recordOutput("Wrist/AtSetpoint", atSetpoint());
    Logger.recordOutput("Wrist/RollerMotorSpeed", rollerMotor.getAppliedOutput());
    Logger.recordOutput("Wrist/RollerMotorVoltage", rollerMotor.getBusVoltage());
  }
}
