package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor1 = new TalonFX(Elevator.LEFT_MOTOR_ID, "rio");
  private final TalonFX elevatorMotor2 = new TalonFX(Elevator.RIGHT_MOTOR_ID, "rio");

  private final ProfiledPIDController pidController;
  private double pidOutput;
  private double feedforward;

  private boolean PIDEnabled = false;

  private final ElevatorFeedforward m_ElevatorFeedforward =
      new ElevatorFeedforward(Elevator.KS, Elevator.KG, Elevator.KV, Elevator.KA);

  private double setpoint;

  public ElevatorSubsystem() {
    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    fx_cfg.Slot0.kV = Elevator.MOTOR_KV;
    fx_cfg.Slot0.kP = Elevator.MOTOR_KP;
    fx_cfg.Slot0.kI = Elevator.MOTOR_KI;
    fx_cfg.Slot0.kD = Elevator.MOTOR_KD;
    fx_cfg.Slot0.kS = Elevator.MOTOR_KS;

    fx_cfg.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Elevator.MAX_SAFE_POS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0);

    fx_cfg.withSoftwareLimitSwitch(limitSwitchConfigs);

    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor1.getConfigurator().apply(fx_cfg);

    pidController =
        new ProfiledPIDController(
            Elevator.KP,
            Elevator.KI,
            Elevator.KD,
            new TrapezoidProfile.Constraints(Elevator.MAX_VELOCITY, Elevator.MAX_ACCELERATION));

    pidController.setTolerance(Elevator.PID_POSITION_TOLERANCE, Elevator.PID_VELOCITY_TOLERANCE);

    setpoint = getEncoderPosition();

    // Log initial configuration
    Logger.recordOutput("Elevator/Config/MaxSafePos", Elevator.MAX_SAFE_POS);
    Logger.recordOutput("Elevator/Config/PIDEnabled", PIDEnabled);
    Logger.recordOutput("Elevator/Config/InitialSetpoint", setpoint);
  }

  public double getEncoderPosition() {
    return elevatorMotor1.getRotorPosition().getValueAsDouble();
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = Math.max(0, Math.min(Elevator.MAX_SAFE_POS, setpoint));
    pidController.setGoal(this.setpoint);
    Logger.recordOutput("Elevator/Command/SetpointCommand", this.setpoint);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void runPID() {
    this.pidOutput = pidController.calculate(getEncoderPosition());
    this.feedforward = m_ElevatorFeedforward.calculate(pidController.getSetpoint().velocity);
    setClampSpeed(pidOutput + feedforward);
  }

  public void moveManual(double speed) {
    this.disablePID();
    setClampSpeed(speed);
    Logger.recordOutput("Elevator/Command/ManualSpeed", speed);
  }

  private void setClampSpeed(double speed) {
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    elevatorMotor1.set(clampedSpeed);
    Logger.recordOutput("Elevator/Control/ClampSpeed", clampedSpeed);
  }

  public void stop() {
    this.disablePID();
    Logger.recordOutput("Elevator/Command/Stop", true);
  }

  public void enablePID() {
    pidController.reset(getEncoderPosition());
    PIDEnabled = true;
    Logger.recordOutput("Elevator/Control/PIDEnabled", true);
  }

  public void disablePID() {
    PIDEnabled = false;
    setSetpoint(getEncoderPosition());
    setClampSpeed(0);
    Logger.recordOutput("Elevator/Control/PIDEnabled", false);
  }

  public boolean isEnabled() {
    return PIDEnabled;
  }

  public boolean isStalled() {
    double current = elevatorMotor1.getSupplyCurrent().getValueAsDouble();
    double velocity = elevatorMotor1.getRotorVelocity().getValueAsDouble();
    boolean stalled =
        Math.abs(velocity) < Elevator.STALL_VELOCITY_THRESHOLD
            && current > Elevator.STALL_CURRENT_THRESHOLD;
    Logger.recordOutput("Elevator/Status/Stalled", stalled);
    return stalled;
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Elevator/PID/", pidController);

    if (PIDEnabled) {
      runPID();
    }

    // Log outputs using AdvantageKit
    Logger.recordOutput("Elevator/Status/Setpoint", setpoint);
    Logger.recordOutput("Elevator/Status/Position", getEncoderPosition());
    Logger.recordOutput(
        "Elevator/Status/Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Status/Current", elevatorMotor1.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Status/Temperature", elevatorMotor1.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Elevator/Status/AtSetpoint", atSetpoint());
    Logger.recordOutput("Elevator/Control/PIDOutput", pidOutput);
    Logger.recordOutput("Elevator/Control/Feedforward", feedforward);
    Logger.recordOutput("Elevator/Control/TotalOutput", pidOutput + feedforward);
    Logger.recordOutput(
        "Elevator/Status/Velocity", elevatorMotor1.getRotorVelocity().getValueAsDouble());
  }
}
