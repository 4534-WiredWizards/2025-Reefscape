package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Wrist.STALL_VELOCITY_THRESHOLD;

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
  private double lastAngle = 0.0;

  // State tracking for logging
  private String currentStatus = "Initialized";
  private double commandedAngle = 0.0;
  private double manualSpeed = 0.0;

  private final SysIdRoutine wristSysId;
  private final ElevatorSubsystem elevator;

  public WristSubsystem(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    // Configure PID controller
    pidController =
        new ProfiledPIDController(
            Wrist.KP,
            Wrist.KI,
            Wrist.KD,
            new TrapezoidProfile.Constraints(Wrist.MAX_VELOCITY, Wrist.MAX_ACCELERATION));
    pidController.setTolerance(Wrist.PID_POSITION_TOLERANCE, Wrist.PID_VELOCITY_TOLERANCE);

    // Configure feedforward
    m_WristFeedforward = new ArmFeedforward(Wrist.KS, Wrist.KG, Wrist.KV, Wrist.KA);

    // Configure motor
    wristMotor = new TalonFX(Wrist.PIVOT_MOTOR_ID);
    configureMotor();

    // Configure absolute encoder
    absEncoder = new DutyCycleEncoder(Wrist.Encoder.PORT);

    // Initialize state
    setpoint = getAngle();
    lastAngle = setpoint;

    // Configure SysId
    wristSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default config
                null, // Use default timeout
                null, // Use default ramp rate
                (state) -> Logger.recordOutput("Wrist/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                null, // No additional logging required
                this));

    // Add commands to dashboard
    SmartDashboard.putData("Wrist/Zero Wrist", zeroCommand());
    SmartDashboard.putData("Wrist PID Controller", pidController);

    // Log initial configuration once at startup
    logConfiguration();
  }

  /** Logs the static configuration parameters once at initialization */
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

  /** Configures the wrist motor with all necessary settings */
  private void configureMotor() {
    var fx_cfg = new TalonFXConfiguration();

    // Calculate software limits
    double forwardLimit = Wrist.MAX_SAFE_ANGLE * Wrist.GEAR_RATIO;
    double reverseLimit = Wrist.MIN_SAFE_ANGLE * Wrist.GEAR_RATIO;

    // Software limit switch configuration
    SoftwareLimitSwitchConfigs limitSwitches =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(forwardLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(reverseLimit);

    // Feedback configuration
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Apply software limits
    fx_cfg.withSoftwareLimitSwitch(limitSwitches);

    // Motor output configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configuration
    wristMotor.getConfigurator().apply(fx_cfg);

    // Log soft limits for debugging
    Logger.recordOutput("Wrist/Config/SoftForwardLimit", forwardLimit);
    Logger.recordOutput("Wrist/Config/SoftReverseLimit", reverseLimit);
  }

  /**
   * Gets the current angle of the wrist
   *
   * @return The current angle in degrees
   */
  public double getAngle() {
    double motorRotations = wristMotor.getRotorPosition().getValueAsDouble();
    double wristRotations = motorRotations / Wrist.GEAR_RATIO;
    return (wristRotations * 360) + Wrist.TRUE_ZERO;
  }

  /**
   * Gets the velocity of the wrist
   *
   * @return The current velocity in degrees per second
   */
  public double getVelocity() {
    double motorVelocity = wristMotor.getRotorVelocity().getValueAsDouble();
    return (motorVelocity / Wrist.GEAR_RATIO) * 360;
  }

  /**
   * Sets the wrist to a specific angle
   *
   * @param angle The desired angle in degrees
   */
  public void setWristSetpoint(double angle) {
    // Safety check against elevator position
    this.setpoint = angle;
    commandedAngle = angle;
    pidController.setGoal(this.setpoint);
    Logger.recordOutput("Wrist/Status/Setpoint", this.setpoint);
  }

  /** Runs the PID controller to maintain the current setpoint */
  public void runPID() {
    double currentAngle = getAngle();
    this.pidOutput = pidController.calculate(currentAngle);
    this.feedforward =
        m_WristFeedforward.calculate(
            Units.degreesToRadians(pidController.getSetpoint().position),
            Units.degreesToRadians(pidController.getSetpoint().velocity));

    double totalOutput = pidOutput;
    setClampSpeed(totalOutput);

    currentStatus = "PID Control";
  }

  /**
   * Manually move the wrist at a specified speed
   *
   * @param speed Speed between -1.0 and 1.0
   */
  public void moveManual(double speed) {
    disablePID();
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    setClampSpeed(clampedSpeed);

    manualSpeed = speed;
    currentStatus = "Manual Control";
  }

  /**
   * Sets a clamped motor speed
   *
   * @param speed Speed between -1.0 and 1.0
   */
  private void setClampSpeed(double speed) {
    double newSpeed = Math.max(-1, Math.min(1, speed));
    wristMotor.set(newSpeed);
    Logger.recordOutput("Wrist/Control/MotorOutput", newSpeed);
  }

  /** Stops the wrist motor */
  public void stop() {
    disablePID();
    wristMotor.stopMotor();
    currentStatus = "Stopped";
  }

  /**
   * Creates a command to stop the wrist
   *
   * @return A command that stops the wrist
   */
  public Command Stop() {
    return Commands.runOnce(
        () -> {
          stop();
        },
        this);
  }

  /** Enables PID control for the wrist */
  public void enablePID() {
    pidController.reset(getAngle());
    isPIDEnabled = true;
    currentStatus = "PID Enabled";
  }

  /** Disables PID control for the wrist */
  public void disablePID() {
    isPIDEnabled = false;
    setpoint = getAngle();
    wristMotor.stopMotor();
    pidOutput = 0;
    feedforward = 0;
    currentStatus = "PID Disabled";
  }

  /**
   * Checks if PID control is enabled
   *
   * @return True if PID is enabled, false otherwise
   */
  public boolean isPIDEnabled() {
    return isPIDEnabled;
  }

  /**
   * Checks if the wrist is at the target position
   *
   * @return True if at setpoint, false otherwise
   */
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  /**
   * Creates a command to set the current position as zero
   *
   * @return A command that sets the current position as zero
   */
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

  /**
   * Command to zero the wrist by moving it down until stall
   *
   * @return A command that zeros the wrist
   */
  public Command zeroCommand() {
    return Commands.sequence(
        // Start zeroing process
        Commands.runOnce(
            () -> {
              isZeroing = true;
              currentStatus = "Zeroing started";
            }),
        // Move wrist until stalled
        Commands.deadline(
                Commands.waitUntil(this::isStalled),
                Commands.run(() -> moveManual(-Wrist.ZEROING_SPEED), this))
            .handleInterrupt(
                () -> {
                  isZeroing = false;
                  stop();
                }),
        // Set current position as zero
        setZeroCommand());
  }

  /**
   * Command to move wrist to a specific angle
   *
   * @param angle The desired angle
   * @return A command that moves the wrist to the angle
   */
  public Command moveToAngleCommand(double angle) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              enablePID();
              setWristSetpoint(angle);
              currentStatus = "Moving to angle: " + angle;
            },
            this),
        Commands.waitUntil(this::atSetpoint),
        Commands.runOnce(() -> currentStatus = "At angle: " + angle));
  }

  /**
   * Enhanced stall detection for zeroing
   *
   * @return True if stalled for a certain number of cycles, false otherwise
   */
  public boolean isStalled() {
    return stallCount >= STALL_COUNT_THRESHOLD;
  }

  /**
   * Checks if the wrist has been zeroed
   *
   * @return True if zeroed, false otherwise
   */
  public boolean isZeroed() {
    return isZeroed;
  }

  /**
   * Runs the wrist motor for system identification
   *
   * @param volts Voltage to apply to the motor
   */
  private void runCharacterization(double volts) {
    wristMotor.set(volts / 12.0); // Convert volts to a -1 to 1 motor output
    Logger.recordOutput("Wrist/SysId/Voltage", volts);
    currentStatus = "Running SysId characterization";
  }

  /**
   * Command for quasi-static system identification
   *
   * @param direction The direction to run the test
   * @return A command that runs the SysId test
   */
  public Command sysIdCommand(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(wristSysId.quasistatic(direction));
  }

  /**
   * Command for dynamic system identification
   *
   * @param direction The direction to run the test
   * @return A command that runs the SysId test
   */
  public Command dynamicSysIdCommand(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(wristSysId.dynamic(direction));
  }

  @Override
  public void periodic() {
    // Get current values once to avoid multiple hardware calls
    double currentAngle = getAngle();
    double velocity = getVelocity();
    double voltage = wristMotor.getMotorVoltage().getValueAsDouble();
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    double temperature = wristMotor.getDeviceTemp().getValueAsDouble();
    double motorPosition = wristMotor.getRotorPosition().getValueAsDouble();
    double absEncoderValue = absEncoder.get();

    // Safety check for elevator interaction

    // Run PID if enabled
    if (isPIDEnabled) {
      runPID();
    }

    boolean voltageApplied = Math.abs(voltage) > 0.1;
    boolean velocityLow = Math.abs(velocity) < STALL_VELOCITY_THRESHOLD;
    boolean farPIDSetpoint = Math.abs(setpoint - getAngle()) > Wrist.PID_POSITION_TOLERANCE;

    // Stall detection logic
    boolean potentialStall =
        (voltageApplied && velocityLow && !isPIDEnabled)
            || (voltageApplied && velocityLow && isPIDEnabled && farPIDSetpoint);

    if (potentialStall) {
      stallCount++;

      // If stalled and not in zeroing process, stop the motor
      if (isStalled() && !isZeroing) {
        stop();
        currentStatus = "Stalled - motor stopped";
      }
    } else {
      stallCount = 0;
    }

    // Log all data in structured categories
    logStatusData(
        currentAngle, velocity, voltage, current, temperature, motorPosition, absEncoderValue);
    logCommandData();
    logPIDData();
    logStallData(potentialStall);

    // Log angle error when in position control mode
    if (currentStatus.contains("Moving to angle")
        || currentStatus.contains("At angle")
        || isPIDEnabled) {
      Logger.recordOutput("Wrist/Status/AngleError", setpoint - currentAngle);
    }

    // Track for next iteration
    lastAngle = currentAngle;
  }

  /** Logs basic status data about the wrist */
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

  /** Logs command-related data */
  private void logCommandData() {
    Logger.recordOutput("Wrist/Command/Angle", commandedAngle);
    Logger.recordOutput("Wrist/Command/ManualSpeed", manualSpeed);
  }

  /** Logs PID and feedforward related data */
  private void logPIDData() {
    Logger.recordOutput("Wrist/Control/PIDEnabled", isPIDEnabled);
    Logger.recordOutput("Wrist/Control/PIDOutput", pidOutput);
    Logger.recordOutput("Wrist/Control/FeedForward", feedforward);
    Logger.recordOutput("Wrist/Control/TotalOutput", pidOutput + feedforward);

    if (isPIDEnabled) {
      Logger.recordOutput(
          "Wrist/Control/PIDSetpointPosition", pidController.getSetpoint().position);
      Logger.recordOutput(
          "Wrist/Control/PIDSetpointVelocity", pidController.getSetpoint().velocity);
    }
  }

  /** Logs stall detection related data */
  private void logStallData(boolean potentialStall) {
    Logger.recordOutput("Wrist/Status/StallDetectionActive", potentialStall);
    Logger.recordOutput("Wrist/Status/StallCount", stallCount);
    Logger.recordOutput("Wrist/Status/IsStalled", isStalled());
  }
}
