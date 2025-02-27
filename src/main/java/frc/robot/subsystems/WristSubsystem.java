package frc.robot.subsystems;

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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Wrist;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final DutyCycleEncoder absEncoder;

  private final ProfiledPIDController pidController;
  private boolean PIDEnabled = false;

  private final ArmFeedforward m_WristFeedforward =
      new ArmFeedforward(Wrist.KS, Wrist.KG, Wrist.KV, Wrist.KA);

  private double setpoint;
  private double pidOutput = 0;
  private double feedforward = 0;

  private SysIdRoutine wristSysId;

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

    double forwardLimit = Wrist.MAX_SAFE_ANGLE * Wrist.GEAR_RATIO;
    double reverseLimit = Wrist.MIN_SAFE_ANGLE * Wrist.GEAR_RATIO;

    // Software limit switch configuration
    SoftwareLimitSwitchConfigs limitSwitches =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(forwardLimit)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(reverseLimit);

    Logger.recordOutput("Wrist/Config/SoftForwardLimit", forwardLimit);
    Logger.recordOutput("Wrist/Config/SoftReverseLimit", reverseLimit);

    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fx_cfg.withSoftwareLimitSwitch(limitSwitches);
    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wristMotor = new TalonFX(Wrist.PIVOT_MOTOR_ID);
    wristMotor.getConfigurator().apply(fx_cfg);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

    absEncoder = new DutyCycleEncoder(Wrist.Encoder.PORT);

    setpoint = getAngle();


    // Configure SysId
    wristSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
              null, // Use default config
              null, // Use default timeout
              null, // Use default ramp rate
              (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
              (voltage) -> runCharacterization(voltage.in(Volts)),
              null, // No additional logging required
              this));

    Logger.recordOutput("Wrist/Status/Setpoint", setpoint);

    // Log initial configuration
    Logger.recordOutput("Wrist/Config/PIDEnabled", PIDEnabled);
    Logger.recordOutput("Wrist/Config/InitialSetpoint", setpoint);
    Logger.recordOutput("Wrist/Config/GearRatio", Wrist.GEAR_RATIO);
    SmartDashboard.putData("Wrist/pidController", pidController);
  }

  public double getAngle() {
    double motorRotations = wristMotor.getRotorPosition().getValueAsDouble();
    double wristRotations = motorRotations / Wrist.GEAR_RATIO;
    return (wristRotations * 360) + Wrist.TRUE_ZERO;
  }

  public void setWristSetpoint(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Wrist/Status/Setpoint", this.setpoint);
    pidController.setGoal(this.setpoint);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void runPID() {
    this.pidOutput = pidController.calculate(getAngle());
    this.feedforward =
        m_WristFeedforward.calculate(
            Units.degreesToRadians(pidController.getSetpoint().position),
            Units.degreesToRadians(pidController.getSetpoint().velocity));
    setClampSpeed(pidOutput); //  Uncomment the feedforward
  }

  public void moveManual(double speed) {
    this.disablePID();
    setClampSpeed(speed);
    Logger.recordOutput("Wrist/Command/ManualSpeed", speed);
  }

  private void setClampSpeed(double speed) {
    double newSpeed = Math.max(-1, Math.min(1, speed));
    wristMotor.set(newSpeed);
    Logger.recordOutput("Wrist/Control/ClampSpeed", newSpeed);
  }

  public void stop() {
    this.disablePID();
    Logger.recordOutput("Wrist/Command/Stop", true);
  }

  public boolean isStalled() {
    double current = wristMotor.getSupplyCurrent().getValueAsDouble();
    double velocity = wristMotor.getRotorVelocity().getValueAsDouble();
    boolean stalled =
        Math.abs(velocity) < Wrist.STALL_VELOCITY_THRESHOLD
            && Math.abs(current) > Wrist.STALL_CURRENT_THRESHOLD;
    Logger.recordOutput("Wrist/Status/Stalled", stalled);
    return stalled;
  }

  public void enablePID() {
    pidController.reset(getAngle());
    PIDEnabled = true;
    Logger.recordOutput("Wrist/Control/PIDEnabled", true);
  }

  public void disablePID() {
    PIDEnabled = false;
    setWristSetpoint(getAngle());
    setClampSpeed(0);
    pidOutput = 0;
    feedforward = 0;
    Logger.recordOutput("Wrist/Control/PIDEnabled", false);
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  
  private void runCharacterization(double volts) {
    wristMotor.set(volts / 12.0); // Convert volts to a -1 to 1 motor output
    Logger.recordOutput("Wrist/SysIdVoltage", volts);
}

// Call this method in your autoroutine to trigger characterization
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
    if (PIDEnabled) {
      runPID();
    }

    // Log outputs using AdvantageKit
    Logger.recordOutput("Wrist/Status/CurrentAngle", getAngle());
    Logger.recordOutput(
        "Wrist/Status/RawEncoderValue", wristMotor.getRotorPosition().getValueAsDouble());
    Logger.recordOutput("Wrist/Status/AbsoluteEncoderValue", absEncoder.get());
    Logger.recordOutput("Wrist/Status/Voltage", wristMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Wrist/Control/PidOutput", pidOutput);
    Logger.recordOutput("Wrist/Control/PidOutputVel", pidController.getSetpoint().velocity);
    Logger.recordOutput("Wrist/Control/PidOutputPos", pidController.getSetpoint().position);
    Logger.recordOutput("Wrist/Control/FeedForward", feedforward);
    Logger.recordOutput("Wrist/Control/TotalMotorOutput", pidOutput + feedforward);
    Logger.recordOutput("Wrist/Status/AtSetpoint", atSetpoint());
    Logger.recordOutput("Wrist/IsCoralRange", getAngle() > Wrist.CORAL_MAX_ANGLE);
  }
}
