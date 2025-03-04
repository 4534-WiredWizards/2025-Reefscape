package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Wrist;
import static frc.robot.Constants.Wrist.STALL_VELOCITY_THRESHOLD;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder absEncoder;

  private final ProfiledPIDController pidController;
  private final ArmFeedforward m_WristFeedforward;

  private double setpoint;
  private double pidOutput = 0;
  private double feedforward = 0;
  private boolean isPIDEnabled = false;
  private boolean isZeroed = false;
  private boolean isZeroing = false;
  private int stallCount = 0;
  private static final int STALL_COUNT_THRESHOLD = 10;

  // State tracking for logging
  private String currentStatus = "Initialized";
  private double commandedAngle = 0.0;
  private double manualSpeed = 0.0;

  private final SysIdRoutine wristSysId;
  private final ElevatorSubsystem elevator;

  public WristSubsystem(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    pidController =
        new ProfiledPIDController(
            Wrist.KP,
            Wrist.KI,
            Wrist.KD,
            new TrapezoidProfile.Constraints(Wrist.MAX_VELOCITY, Wrist.MAX_ACCELERATION));
    pidController.setTolerance(Wrist.PID_POSITION_TOLERANCE, Wrist.PID_VELOCITY_TOLERANCE);

    m_WristFeedforward = new ArmFeedforward(Wrist.KS, Wrist.KG, Wrist.KV, Wrist.KA);

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
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null,
                this));

    SmartDashboard.putData("Wrist/Zero Wrist", zeroCommand());
    SmartDashboard.putData("Wrist PID Controller", pidController);

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

    double forwardLimit = Wrist.MAX_SAFE_ANGLE * Wrist.GEAR_RATIO;
    double reverseLimit = Wrist.MIN_SAFE_ANGLE * Wrist.GEAR_RATIO;

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

    wristMotor.getConfigurator().apply(fx_cfg);

    Logger.recordOutput("Wrist/Config/SoftForwardLimit", forwardLimit);
    Logger.recordOutput("Wrist/Config/SoftReverseLimit", reverseLimit);
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
    this.setpoint = angle;
    commandedAngle = angle;
    pidController.setGoal(this.setpoint);
    Logger.recordOutput("Wrist/Status/Setpoint", this.setpoint);
  }

  public void runPID() {
    double currentAngle = getAngle();
    this.pidOutput = pidController.calculate(currentAngle);
    this.feedforward =
        m_WristFeedforward.calculate(
            Units.degreesToRadians(pidController.getSetpoint().position),
            Units.degreesToRadians(pidController.getSetpoint().velocity));

    setClampSpeed(pidOutput + feedforward);
    currentStatus = "PID Control";
  }

  public void moveManual(double speed) {
    disablePID();
    setClampSpeed(Math.max(-1, Math.min(1, speed)));
    manualSpeed = speed;
    currentStatus = "Manual Control";
  }

  private void setClampSpeed(double speed) {
    wristMotor.set(Math.max(-1, Math.min(1, speed)));
    Logger.recordOutput("Wrist/Control/MotorOutput", speed);
  }

  public void stop() {
    disablePID();
    wristMotor.stopMotor();
    currentStatus = "Stopped";
  }

  public Command Stop() {
    return Commands.runOnce(this::stop, this);
  }

  public void enablePID() {
    pidController.reset(getAngle());
    isPIDEnabled = true;
    currentStatus = "PID Enabled";
  }

  public void disablePID() {
    isPIDEnabled = false;
    setpoint = getAngle();
    wristMotor.stopMotor();
    pidOutput = 0;
    feedforward = 0;
    currentStatus = "PID Disabled";
  }

  public boolean isPIDEnabled() {
    return isPIDEnabled;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
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
                Commands.run(() -> moveManual(-Wrist.ZEROING_SPEED), this))
            .handleInterrupt(
                () -> {
                  isZeroing = false;
                  stop();
                }),
        setZeroCommand());
  }

  public Command moveToAngleCommand(double angle) {
    return Commands.sequence(
        Commands.runOnce(() -> {
          enablePID();
          setWristSetpoint(angle);
          currentStatus = "Moving to angle: " + angle;
        }, this),
        Commands.waitUntil(this::atSetpoint),
        Commands.runOnce(() -> currentStatus = "At angle: " + angle));
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

    if (isPIDEnabled) {
      runPID();
    }

    // Modified stall detection (only active during zeroing)
    if (isZeroing) {
        boolean potentialStall = Math.abs(voltage) > 0.1 
                               && Math.abs(velocity) < STALL_VELOCITY_THRESHOLD;
        if (potentialStall) {
            stallCount++;
        } else {
            stallCount = 0;
        }
    } else {
        stallCount = 0; // Reset when not zeroing
    }

    logStatusData(currentAngle, velocity, voltage, current, temperature, motorPosition, absEncoderValue);
    logCommandData();
    logPIDData();
    logStallData(isZeroing && (stallCount > 0));

    if (currentStatus.contains("Moving to angle") 
        || currentStatus.contains("At angle")
        || isPIDEnabled) {
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
    Logger.recordOutput("Wrist/Command/ManualSpeed", manualSpeed);
  }

  private void logPIDData() {
    Logger.recordOutput("Wrist/Control/PIDEnabled", isPIDEnabled);
    Logger.recordOutput("Wrist/Control/PIDOutput", pidOutput);
    Logger.recordOutput("Wrist/Control/FeedForward", feedforward);
    Logger.recordOutput("Wrist/Control/TotalOutput", pidOutput + feedforward);

    if (isPIDEnabled) {
      Logger.recordOutput("Wrist/Control/PIDSetpointPosition", pidController.getSetpoint().position);
      Logger.recordOutput("Wrist/Control/PIDSetpointVelocity", pidController.getSetpoint().velocity);
    }
  }

  private void logStallData(boolean potentialStall) {
    Logger.recordOutput("Wrist/Status/StallDetectionActive", potentialStall);
    Logger.recordOutput("Wrist/Status/StallCount", stallCount);
    Logger.recordOutput("Wrist/Status/IsStalled", isStalled());
  }
}