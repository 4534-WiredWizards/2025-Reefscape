package frc.robot.subsystems;

import static frc.robot.Constants.Elevator.MAX_SAFE_POS;
import static frc.robot.Constants.Elevator.MIN_SAFE_POS;
import static frc.robot.Constants.Elevator.STALL_VELOCITY_THRESHOLD;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSubsystem extends SubsystemBase {

  // Mecanism 2d
  // Class member variables
  private final LoggedMechanism2d elevatorMechanism;
  private final LoggedMechanismRoot2d elevatorRoot;
  private final LoggedMechanismLigament2d baseLigament;
  private final LoggedMechanismLigament2d elevatorLigament;
  private final double CANVAS_WIDTH = 1.5; // 1.5 meter width to accommodate horizontal extension
  private final double CANVAS_HEIGHT = 2.0; // 2 meter height (covers 6ft max height)
  private final double ELEVATOR_THICKNESS = .05; // 5cm thickness

  private final TalonFX elevatorMotor1 = new TalonFX(Elevator.LEFT_MOTOR_ID, "rio");
  private final TalonFX elevatorMotor2 = new TalonFX(Elevator.RIGHT_MOTOR_ID, "rio");

  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  private double setpoint;
  private boolean isZeroed = false;
  private boolean isZeroing = false;
  private boolean lookForStalled = false;
  private int stallCount = 0;
  private static final int STALL_COUNT_THRESHOLD = 10;

  // State tracking for logging
  private String currentStatus = "Initialized";
  private double commandedPosition = 0.0;
  private double commandedVoltage = 0.0;
  private double manualSpeed = 0.0;

  public ElevatorSubsystem() {
    configureMotors();
    setpoint = getEncoderPosition();

    // Create a Mechanism2d visualization for the elevator
    elevatorMechanism =
        new LoggedMechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT, new Color8Bit(Color.kBlack));
    elevatorRoot = elevatorMechanism.getRoot("ElevatorRoot", CANVAS_WIDTH / 2.0, 0);

    // Create base of the elevator - static part pointing up (90 degrees)
    baseLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "Base",
                Units.inchesToMeters(5),
                90, // 90 degrees = vertical pointing up
                6,
                new Color8Bit(Color.kGray)));

    // Create the moving part of the elevator pointing horizontally (0 degrees) from
    // top of base
    elevatorLigament =
        baseLigament.append(
            new LoggedMechanismLigament2d(
                "Elevator",
                100, // Initial length will be set in periodic
                0, // 0 degrees = horizontal pointing right from top of base
                ELEVATOR_THICKNESS * 100, // Width in pixels
                new Color8Bit(0xBF, 0x57, 0x00) // Brown color
                ));

    // Add the mechanism to SmartDashboard
    SmartDashboard.putData("Elevator/Mechanism", elevatorMechanism);

    // Record mecanism in the logger
    Logger.recordOutput("Elevator", elevatorMechanism);

    logConfiguration();
  }

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

  private void configureMotors() {
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();

    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    fx_cfg.Voltage.PeakForwardVoltage = Elevator.PEAK_FORWARD_VOLTAGE;
    fx_cfg.Voltage.PeakReverseVoltage = Elevator.PEAK_REVERSE_VOLTAGE;
    fx_cfg.CurrentLimits.StatorCurrentLimit = 100;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = 0;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = false;

    fx_cfg.Slot0.kP = Elevator.KP;
    fx_cfg.Slot0.kI = Elevator.KI;
    fx_cfg.Slot0.kD = Elevator.KD;

    fx_cfg.Slot0.kV = Elevator.KV;
    fx_cfg.Slot0.kS = Elevator.KS;
    fx_cfg.Slot0.kA = Elevator.KA;
    fx_cfg.Slot0.kG = Elevator.KG;

    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Elevator.CRUISE_VELOCITY;
    fx_cfg.MotionMagic.MotionMagicAcceleration = Elevator.MAX_ACCELERATION;
    // fx_cfg.MotionMagic.MotionMagicJerk = Elevator.JERK;

    SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Elevator.MAX_SAFE_POS)
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(Elevator.MIN_SAFE_POS);

    fx_cfg.withSoftwareLimitSwitch(limitSwitchConfigs);
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorMotor1.getConfigurator().apply(fx_cfg);
    elevatorMotor1.stopMotor();
    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    SmartDashboard.putData("Elevator/Zero Elevator", zeroCommand());
  }

  public void setPosition(double position) {
    double clampedPosition = Math.max(MIN_SAFE_POS, Math.min(MAX_SAFE_POS, position));
    setpoint = clampedPosition;
    elevatorMotor1.setControl(positionVoltage.withPosition(position));

    commandedPosition = position;
    currentStatus = "Moving to position";
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getEncoderPosition() {
    return elevatorMotor1.getRotorPosition().getValueAsDouble();
  }

  public double getSpeed() {
    return elevatorMotor1.getRotorVelocity().getValueAsDouble();
  }

  public void moveManual(double speed) {
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    elevatorMotor1.set(clampedSpeed);

    manualSpeed = speed;
    currentStatus = "Manual control";
  }

  public void setVoltage(double voltage) {
    elevatorMotor1.setControl(voltageOut.withOutput(voltage));

    commandedVoltage = voltage;
    currentStatus = "Voltage control";
  }

  public void stop() {
    elevatorMotor1.stopMotor();
    currentStatus = "Stopped";
  }

  public Command Stop() {
    return Commands.runOnce(this::stop, this);
  }

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

  public Command zeroCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isZeroing = true;
              currentStatus = "Zeroing started";
            }),
        Commands.deadline(
                Commands.waitUntil(this::isStalled),
                Commands.run(() -> setVoltage(-Elevator.ZEROING_VOLTAGE), this))
            .handleInterrupt(
                () -> {
                  isZeroing = false;
                  stop();
                }),
        setZeroCommand());
  }

  public Command moveToPositionCommand(double position) {
    return Commands.sequence(
        Commands.runOnce(() -> setPosition(position), this),
        Commands.waitUntil(() -> isAtPosition(position)),
        Commands.runOnce(() -> currentStatus = "At position: " + position));
  }

  public boolean isAtPosition(double targetPosition) {
    double currentPosition = getEncoderPosition();
    return Math.abs(currentPosition - targetPosition) < Elevator.POSITION_TOLERANCE;
  }

  public boolean isStalled() {
    return stallCount >= STALL_COUNT_THRESHOLD;
  }

  public boolean isZeroed() {
    return isZeroed;
  }

  public void setLookForStalled(boolean lookForStalled) {
    this.lookForStalled = lookForStalled;
  }

  @Override
  public void periodic() {
    double currentPosition = getEncoderPosition();
    double velocity = elevatorMotor1.getRotorVelocity().getValueAsDouble();
    double voltage = elevatorMotor1.getMotorVoltage().getValueAsDouble();
    double current = elevatorMotor1.getSupplyCurrent().getValueAsDouble();
    double temperature = elevatorMotor1.getDeviceTemp().getValueAsDouble();

    // Modified stall detection logic (only active during zeroing)
    boolean potentialStall = false;
    if (isZeroing || lookForStalled) {
      potentialStall = Math.abs(voltage) > 0.1 && Math.abs(velocity) < STALL_VELOCITY_THRESHOLD;

      if (potentialStall) {
        stallCount++;
      } else {
        stallCount = 0;
      }
    } else {
      stallCount = 0; // Reset stall count when not zeroing
    }

    logStatusData(currentPosition, velocity, voltage, current, temperature);
    logCommandData();
    logStallData(potentialStall);

    if (currentStatus.contains("Moving to position") || currentStatus.contains("At position")) {
      Logger.recordOutput("Elevator/Status/PositionError", setpoint - currentPosition);
    }

    // Mechanism 2d visualization
    double elevatorExtension = currentPosition;
    elevatorLigament.setLength(Units.inchesToMeters(elevatorExtension));

    // Add color coding based on different states
    if (isStalled()) {
      elevatorLigament.setColor(new Color8Bit(Color.kRed));
    } else if (voltage != 0.0) {
      elevatorLigament.setColor(new Color8Bit(Color.kYellow)); // Moving
    } else {
      elevatorLigament.setColor(new Color8Bit(0xBF, 0x57, 0x00)); // Default brown
    }
  }

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
    Logger.recordOutput("Elevator/Status/IsAtPosition", isAtPosition(setpoint));
    // setLookForStalled
    Logger.recordOutput("Elevator/Status/lookForStalled", lookForStalled);
  }

  private void logCommandData() {
    Logger.recordOutput("Elevator/Command/Position", commandedPosition);
    Logger.recordOutput("Elevator/Command/Voltage", commandedVoltage);
    Logger.recordOutput("Elevator/Command/ManualSpeed", manualSpeed);
  }

  private void logStallData(boolean potentialStall) {
    Logger.recordOutput("Elevator/Status/StallDetectionActive", potentialStall);
    Logger.recordOutput("Elevator/Status/StallCount", stallCount);
    Logger.recordOutput("Elevator/Status/IsStalled", isStalled());
  }
}
