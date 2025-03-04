package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import static frc.robot.Constants.Elevator.MAX_SAFE_POS;
import static frc.robot.Constants.Elevator.MIN_SAFE_POS;
import static frc.robot.Constants.Elevator.STALL_VELOCITY_THRESHOLD;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor1 = new TalonFX(Elevator.LEFT_MOTOR_ID, "rio");
  private final TalonFX elevatorMotor2 = new TalonFX(Elevator.RIGHT_MOTOR_ID, "rio");

  private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  private double setpoint;
  private boolean isZeroed = false;
  private boolean isZeroing = false;
  private int stallCount = 0;
  private static final int STALL_COUNT_THRESHOLD = 10;
  private double lastPosition = 0.0;

  // State tracking for logging
  private String currentStatus = "Initialized";
  private double commandedPosition = 0.0;
  private double commandedVoltage = 0.0;
  private double manualSpeed = 0.0;

  public ElevatorSubsystem() {
    configureMotors();
    setpoint = getEncoderPosition();
    lastPosition = setpoint;

    // Log initial configuration once at startup
    logConfiguration();
  }

  /** Logs the static configuration parameters once at initialization */
  private void logConfiguration() {
    Logger.recordOutput("Elevator/Config/MaxSafePos", MAX_SAFE_POS);
    Logger.recordOutput("Elevator/Config/MinSafePos", MIN_SAFE_POS);
    Logger.recordOutput("Elevator/Config/PositionTolerance", Elevator.POSITION_TOLERANCE);
    Logger.recordOutput("Elevator/Config/ZeroingVoltage", Elevator.ZEROING_VOLTAGE);
    Logger.recordOutput("Elevator/Config/PID/kP", Elevator.KP);
    Logger.recordOutput("Elevator/Config/PID/kI", Elevator.KI);
    Logger.recordOutput("Elevator/Config/PID/kD", Elevator.KD);
    Logger.recordOutput("Elevator/Config/FF/kV", Elevator.KV);
    Logger.recordOutput("Elevator/Config/FF/kS", Elevator.KS);
    Logger.recordOutput("Elevator/Config/FF/kA", Elevator.KA);
    Logger.recordOutput("Elevator/Config/FF/kG", Elevator.KG);
  }

  /** Configures the elevator motors with all necessary settings */
  private void configureMotors() {
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

    // Feedback configuration
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Voltage configuration
    fx_cfg.Voltage.PeakForwardVoltage = Elevator.PEAK_FORWARD_VOLTAGE;
    fx_cfg.Voltage.PeakReverseVoltage = Elevator.PEAK_REVERSE_VOLTAGE;

    // PID and feedforward configuration
    fx_cfg.Slot0.kP = Elevator.KP;
    fx_cfg.Slot0.kI = Elevator.KI;
    fx_cfg.Slot0.kD = Elevator.KD;

    fx_cfg.Slot0.kV = Elevator.KV;
    fx_cfg.Slot0.kS = Elevator.KS;
    fx_cfg.Slot0.kA = Elevator.KA;
    fx_cfg.Slot0.kG = Elevator.KG;

    // Motion Magic Configuration

    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Elevator.CRUISE_VELOCITY;
    fx_cfg.MotionMagic.MotionMagicAcceleration = Elevator.MAX_ACCELERATION;
    // fx_cfg.MotionMagic.MotionMagicJerk = Elevator.JERK;

    // Soft limits configuration
    SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Elevator.MAX_SAFE_POS)
            .withReverseSoftLimitEnable(
                false) // Disable reverse soft limit to enable stall zeroing && to prevent bottom
            // shake due to soft limit
            .withReverseSoftLimitThreshold(Elevator.MIN_SAFE_POS);

    fx_cfg.withSoftwareLimitSwitch(limitSwitchConfigs);

    // Motor output configuration
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configuration to primary motor
    elevatorMotor1.getConfigurator().apply(fx_cfg);

    // Stop Main Motor
    elevatorMotor1.stopMotor();

    // Configure right motor to follow left motor
    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    // Add the zeroings command to smart dashboard
    SmartDashboard.putData("Elevator/Zero Elevator", zeroCommand());
  }

  /**
   * Sets the elevator to a specific position
   *
   * @param position The desired encoder position
   */
  public void setPosition(double position) {
    double clampedPosition = Math.max(MIN_SAFE_POS, Math.min(MAX_SAFE_POS, position));
    setpoint = clampedPosition;
    elevatorMotor1.setControl(positionVoltage.withPosition(position));

    // Track for logging in periodic
    commandedPosition = position;
    currentStatus = "Moving to position";
  }

  /**
   * Retrieves the current setpoint of the elevator subsystem.
   *
   * @return The current setpoint value.
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the current encoder position of the elevator
   *
   * @return The current encoder position
   */
  public double getEncoderPosition() {
    return elevatorMotor1.getRotorPosition().getValueAsDouble();
  }

  /**
   * Gets the current speed (velocity) of the elevator
   *
   * @return The current speed in units per second
   */
  public double getSpeed() {
    return elevatorMotor1.getRotorVelocity().getValueAsDouble();
  }

  /**
   * Manually move the elevator at a specified speed
   *
   * @param speed Speed between -1.0 and 1.0
   */
  public void moveManual(double speed) {
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    elevatorMotor1.set(clampedSpeed);

    // Track for logging in periodic
    manualSpeed = speed;
    currentStatus = "Manual control";
  }

  /**
   * Move elevator at a specific voltage
   *
   * @param voltage The voltage to apply to the motor
   */
  public void setVoltage(double voltage) {
    elevatorMotor1.setControl(voltageOut.withOutput(voltage));

    // Track for logging in periodic
    commandedVoltage = voltage;
    currentStatus = "Voltage control";
  }

  /** Stops the elevator motors */
  public void stop() {
    elevatorMotor1.stopMotor();
    currentStatus = "Stopped";
  }

  // Public motor stopping command
  public Command Stop() {
    return Commands.runOnce(
        () -> {
          stop();
        },
        this);
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
              elevatorMotor1.setPosition(0);
              isZeroing = false;
              stop();
              currentStatus = "Zero set";
            })
        .ignoringDisable(true);
  }

  /**
   * Command to zero the elevator by moving it down until stall
   *
   * @return A command that zeros the elevator
   */
  public Command zeroCommand() {
    return Commands.sequence(
        // Start zeroing process
        Commands.runOnce(
            () -> {
              isZeroing = true;
              currentStatus = "Zeroing started";
            }),
        // Lower elevator until stalled
        Commands.deadline(
                Commands.waitUntil(this::isStalled),
                Commands.run(() -> setVoltage(-Elevator.ZEROING_VOLTAGE), this))
            .handleInterrupt(
                () -> {
                  isZeroing = false;
                  stop();
                }),
        // Set current position as zero
        setZeroCommand());
  }

  /**
   * Command to move elevator to a specific position
   *
   * @param position The desired position
   * @return A command that moves the elevator to the position
   */
  public Command moveToPositionCommand(double position) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              setPosition(position);
              currentStatus = "Moving to position: " + position;
            },
            this),
        Commands.waitUntil(() -> isAtPosition(position)),
        Commands.runOnce(() -> currentStatus = "At position: " + position));
  }

  /**
   * Checks if the elevator has reached the target position
   *
   * @param targetPosition The target position
   * @return True if at position, false otherwise
   */
  public boolean isAtPosition(double targetPosition) {
    double currentPosition = getEncoderPosition();
    double error = Math.abs(currentPosition - targetPosition);
    return error < Elevator.POSITION_TOLERANCE;
  }

  /**
   * Checks if the elevator is stalled
   *
   * @return True if stalled, false otherwise
   */
  public boolean isStalled() {
    return stallCount >= STALL_COUNT_THRESHOLD;
  }

  /**
   * Checks if the elevator has been zeroed
   *
   * @return True if zeroed, false otherwise
   */
  public boolean isZeroed() {
    return isZeroed;
  }

  @Override
  public void periodic() {
    // Get current values once to avoid multiple hardware calls
    double currentPosition = getEncoderPosition();
    double velocity = elevatorMotor1.getRotorVelocity().getValueAsDouble();
    double voltage = elevatorMotor1.getMotorVoltage().getValueAsDouble();
    double current = elevatorMotor1.getSupplyCurrent().getValueAsDouble();
    double temperature = elevatorMotor1.getDeviceTemp().getValueAsDouble();

    // Stall detection logic
    boolean potentialStall =
        Math.abs(voltage) > 0.1 && Math.abs(velocity) < STALL_VELOCITY_THRESHOLD;

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
    logStatusData(currentPosition, velocity, voltage, current, temperature);
    logCommandData();
    logStallData(potentialStall);

    // Log position error when in position control mode
    if (currentStatus.contains("Moving to position") || currentStatus.contains("At position")) {
      Logger.recordOutput("Elevator/Status/PositionError", setpoint - currentPosition);
    }
  }

  /** Logs basic status data about the elevator */
  private void logStatusData(
      double position, double velocity, double voltage, double current, double temperature) {
    Logger.recordOutput("Elevator/Status/Text", currentStatus);
    Logger.recordOutput("Elevator/Status/Position", position);
    Logger.recordOutput("Elevator/Status/Velocity", velocity);
    Logger.recordOutput("Elevator/Status/Voltage", voltage);
    Logger.recordOutput("Elevator/Status/Current", current);
    Logger.recordOutput("Elevator/Status/Temperature", temperature);
    Logger.recordOutput("Elevator/Status/Setpoint", setpoint);
    Logger.recordOutput("Elevator/Status/IsZeroed", isZeroed);
    Logger.recordOutput("Elevator/Status/IsZeroing", isZeroing);
  }

  /** Logs command-related data */
  private void logCommandData() {
    Logger.recordOutput("Elevator/Command/Position", commandedPosition);
    Logger.recordOutput("Elevator/Command/Voltage", commandedVoltage);
    Logger.recordOutput("Elevator/Command/ManualSpeed", manualSpeed);
  }

  /** Logs stall detection related data */
  private void logStallData(boolean potentialStall) {
    Logger.recordOutput("Elevator/Status/StallDetectionActive", potentialStall);
    Logger.recordOutput("Elevator/Status/StallCount", stallCount);
    Logger.recordOutput("Elevator/Status/IsStalled", isStalled());
  }
}
