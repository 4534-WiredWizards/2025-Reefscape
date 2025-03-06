package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Wrist.STALL_VELOCITY_THRESHOLD;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Wrist;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder absEncoder;

  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  private double setpoint;
  private boolean isZeroed = false;
  private boolean isZeroing = false;
  private int stallCount = 0;
  private static final int STALL_COUNT_THRESHOLD = 10;

  // State tracking for logging
  private String currentStatus = "Initialized";
  private double commandedAngle = 0.0;
  private double commandedVoltage = 0.0;
  private double manualSpeed = 0.0;

  private final SysIdRoutine wristSysId;
  private final ElevatorSubsystem elevator;

  public WristSubsystem(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    wristMotor = new TalonFX(Wrist.PIVOT_MOTOR_ID);
    configureMotor();

    absEncoder = new DutyCycleEncoder(Wrist.Encoder.PORT);

    setpoint = getAngle();

    wristSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Wrist/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    SmartDashboard.putData("Wrist/Zero Wrist", zeroCommand());

    logConfiguration();
  }

  private void logConfiguration() {
    Logger.recordOutput("Wrist/Config/MaxSafeAngle", Wrist.MAX_SAFE_ANGLE);
    Logger.recordOutput("Wrist/Config/MinSafeAngle", Wrist.MIN_SAFE_ANGLE);
    Logger.recordOutput("Wrist/Config/MinClearElevatorAngle", Wrist.MIN_CLEAR_ELEVATOR_ANGLE);
    Logger.recordOutput("Wrist/Config/GearRatio", Wrist.GEAR_RATIO);
    Logger.recordOutput("Wrist/Config/TrueZero", Wrist.TRUE_ZERO);
    Logger.recordOutput("Wrist/Config/CoralMaxAngle", Wrist.CORAL_MAX_ANGLE);
    Logger.recordOutput("Wrist/Config/ZeroingSpeed", Wrist.ZEROING_SPEED);
    Logger.recordOutput("Wrist/Config/PID/kP", Wrist.KP);
    Logger.recordOutput("Wrist/Config/PID/kI", Wrist.KI);
    Logger.recordOutput("Wrist/Config/PID/kD", Wrist.KD);
    Logger.recordOutput("Wrist/Config/FF/kS", Wrist.KS);
    Logger.recordOutput("Wrist/Config/FF/kG", Wrist.KG);
    Logger.recordOutput("Wrist/Config/FF/kV", Wrist.KV);
    Logger.recordOutput("Wrist/Config/FF/kA", Wrist.KA);
  }

  private void configureMotor() {
    var fx_cfg = new TalonFXConfiguration();

    double forwardLimit = Wrist.MAX_SAFE_ANGLE * Wrist.GEAR_RATIO / 360.0;
    double reverseLimit = Wrist.MIN_SAFE_ANGLE * Wrist.GEAR_RATIO / 360.0;

    SoftwareLimitSwitchConfigs limitSwitches =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(forwardLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(reverseLimit);

    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fx_cfg.withSoftwareLimitSwitch(limitSwitches);
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    fx_cfg.Voltage.PeakForwardVoltage = 12.0;
    fx_cfg.Voltage.PeakReverseVoltage = -12.0;

    // Configure PID and feedforward gains
    fx_cfg.Slot0.kP = Wrist.KP;
    fx_cfg.Slot0.kI = Wrist.KI;
    fx_cfg.Slot0.kD = Wrist.KD;

    // Feedforward gains
    fx_cfg.Slot0.kS = Wrist.KS;
    fx_cfg.Slot0.kG = Wrist.KG;
    fx_cfg.Slot0.kV = Wrist.KV;
    fx_cfg.Slot0.kA = Wrist.KA;

    // Motion Magic settings
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Wrist.CRUISE_VELOCITY / 360.0 * Wrist.GEAR_RATIO;
    fx_cfg.MotionMagic.MotionMagicAcceleration = Wrist.ACCELERATION / 360.0 * Wrist.GEAR_RATIO;

    wristMotor.getConfigurator().apply(fx_cfg);

    Logger.recordOutput("Wrist/Config/SoftForwardLimit", forwardLimit);
    Logger.recordOutput("Wrist/Config/SoftReverseLimit", reverseLimit);
    Logger.recordOutput(
        "Wrist/Config/MotionMagicCruiseVelocity", fx_cfg.MotionMagic.MotionMagicCruiseVelocity);
    Logger.recordOutput(
        "Wrist/Config/MotionMagicAcceleration", fx_cfg.MotionMagic.MotionMagicAcceleration);
  }

  public double getAngle() {
    double motorRotations = wristMotor.getRotorPosition().getValueAsDouble();
    double wristRotations = motorRotations / Wrist.GEAR_RATIO;
    return (wristRotations * 360) + Wrist.TRUE_ZERO;
  }

  public double getVelocity() {
    double motorVelocity = wristMotor.getRotorVelocity().getValueAsDouble();
    return (motorVelocity / Wrist.GEAR_RATIO) * 360;
  }

  public void setWristSetpoint(double angle) {
    double clampedAngle = Math.max(Wrist.MIN_SAFE_ANGLE, Math.min(Wrist.MAX_SAFE_ANGLE, angle));
    this.setpoint = clampedAngle;

    // Convert angle to motor rotations
    double motorPositionRotations = (clampedAngle - Wrist.TRUE_ZERO) / 360.0 * Wrist.GEAR_RATIO;
    wristMotor.setControl(positionVoltage.withPosition(motorPositionRotations));

    commandedAngle = clampedAngle;
    currentStatus = "Moving to angle";

    Logger.recordOutput("Wrist/Status/Setpoint", this.setpoint);
    Logger.recordOutput("Wrist/Status/TargetMotorPosition", motorPositionRotations);
  }

  public void moveManual(double speed) {
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    wristMotor.set(clampedSpeed);
    manualSpeed = speed;
    currentStatus = "Manual Control";
  }

  public void setVoltage(double voltage) {
    wristMotor.setControl(voltageOut.withOutput(voltage));
    commandedVoltage = voltage;
    currentStatus = "Voltage Control";
  }

  public void stop() {
    wristMotor.stopMotor();
    currentStatus = "Stopped";
  }

  public Command Stop() {
    return Commands.runOnce(this::stop, this);
  }

  public Command setZeroCommand() {
    return Commands.runOnce(
            () -> {
              isZeroed = true;
              wristMotor.setPosition(0);
              isZeroing = false;
              stop();
              currentStatus = "Zero set";
            })
        .ignoringDisable(true);
  }

  public Command zeroCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isZeroing = true;
              currentStatus = "Zeroing started";
            }),
        Commands.deadline(
                Commands.waitUntil(this::isStalled),
                Commands.run(() -> setVoltage(-Wrist.ZEROING_SPEED), this))
            .handleInterrupt(
                () -> {
                  isZeroing = false;
                  stop();
                }),
        setZeroCommand());
  }

  public Command moveToAngleCommand(double angle) {
    return Commands.sequence(
        Commands.runOnce(() -> setWristSetpoint(angle), this),
        Commands.waitUntil(() -> isAtPosition(angle)),
        Commands.runOnce(() -> currentStatus = "At angle: " + angle));
  }

  public boolean isAtPosition(double targetAngle) {
    double currentAngle = getAngle();
    return Math.abs(currentAngle - targetAngle) < Wrist.PID_POSITION_TOLERANCE;
  }

  public boolean atSetpoint() {
    return isAtPosition(setpoint);
  }

  public boolean isStalled() {
    return stallCount >= STALL_COUNT_THRESHOLD;
  }

  public boolean isZeroed() {
    return isZeroed;
  }

  private void runCharacterization(double volts) {
    wristMotor.set(volts / 12.0);
    Logger.recordOutput("Wrist/SysId/Voltage", volts);
    currentStatus = "Running SysId characterization";
  }

  public Command sysIdCommand(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(wristSysId.quasistatic(direction));
  }

  public Command dynamicSysIdCommand(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(wristSysId.dynamic(direction));
  }

  @Override
  public void periodic() {
    double currentAngle = getAngle();
    double velocity = getVelocity();
    double voltage = wristMotor.getMotorVoltage().getValueAsDouble();
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    double temperature = wristMotor.getDeviceTemp().getValueAsDouble();
    double motorPosition = wristMotor.getRotorPosition().getValueAsDouble();
    double absEncoderValue = absEncoder.get();

    // Modified stall detection (only active during zeroing)
    if (isZeroing) {
      boolean potentialStall =
          Math.abs(voltage) > 0.1 && Math.abs(velocity) < STALL_VELOCITY_THRESHOLD;
      if (potentialStall) {
        stallCount++;
      } else {
        stallCount = 0;
      }
    } else {
      stallCount = 0; // Reset when not zeroing
    }

    logStatusData(
        currentAngle, velocity, voltage, current, temperature, motorPosition, absEncoderValue);
    logCommandData();
    logStallData(isZeroing && (stallCount > 0));

    if (currentStatus.contains("Moving to angle") || currentStatus.contains("At angle")) {
      Logger.recordOutput("Wrist/Status/AngleError", setpoint - currentAngle);
    }
  }

  private void logStatusData(
      double angle,
      double velocity,
      double voltage,
      double current,
      double temperature,
      double motorPosition,
      double absEncoderValue) {
    Logger.recordOutput("Wrist/Status/Text", currentStatus);
    Logger.recordOutput("Wrist/Status/Angle", angle);
    Logger.recordOutput("Wrist/Status/Velocity", velocity);
    Logger.recordOutput("Wrist/Status/Voltage", voltage);
    Logger.recordOutput("Wrist/Status/Current", current);
    Logger.recordOutput("Wrist/Status/Temperature", temperature);
    Logger.recordOutput("Wrist/Status/Setpoint", setpoint);
    Logger.recordOutput("Wrist/Status/IsZeroed", isZeroed);
    Logger.recordOutput("Wrist/Status/IsZeroing", isZeroing);
    Logger.recordOutput("Wrist/Status/AtSetpoint", atSetpoint());
    Logger.recordOutput("Wrist/Status/RawMotorPosition", motorPosition);
    Logger.recordOutput("Wrist/Status/AbsoluteEncoderValue", absEncoderValue);
    Logger.recordOutput("Wrist/Status/IsInCoralRange", angle > Wrist.CORAL_MAX_ANGLE);
  }

  private void logCommandData() {
    Logger.recordOutput("Wrist/Command/Angle", commandedAngle);
    Logger.recordOutput("Wrist/Command/Voltage", commandedVoltage);
    Logger.recordOutput("Wrist/Command/ManualSpeed", manualSpeed);
  }

  private void logStallData(boolean potentialStall) {
    Logger.recordOutput("Wrist/Status/StallDetectionActive", potentialStall);
    Logger.recordOutput("Wrist/Status/StallCount", stallCount);
    Logger.recordOutput("Wrist/Status/IsStalled", isStalled());
  }
}
