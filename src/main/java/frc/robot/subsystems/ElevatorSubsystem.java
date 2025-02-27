package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor1 = new TalonFX(Elevator.LEFT_MOTOR_ID, "rio");
  private final TalonFX elevatorMotor2 = new TalonFX(Elevator.RIGHT_MOTOR_ID, "rio");

  private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  private double pidOutput;
  private double feedforward;

  private boolean PIDEnabled = false;
  
  

  private double setpoint;

  public ElevatorSubsystem() {
    
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = Elevator.CRUISE_VELOCITY;
    fx_cfg.MotionMagic.MotionMagicAcceleration = Elevator.MAX_ACCELERATION;
    fx_cfg.MotionMagic.MotionMagicJerk = Elevator.JERK;
    fx_cfg.Voltage.PeakForwardVoltage = Elevator.PEAK_FORWARD_VOLTAGE;
    fx_cfg.Voltage.PeakReverseVoltage = Elevator.PEAK_REVERSE_VOLTAGE;

    fx_cfg.Slot0.kV = Elevator.MOTOR_KV;
    fx_cfg.Slot0.kP = Elevator.MOTOR_KP;
    fx_cfg.Slot0.kI = Elevator.MOTOR_KI;
    fx_cfg.Slot0.kD = Elevator.MOTOR_KD;
    fx_cfg.Slot0.kS = Elevator.MOTOR_KS;
    fx_cfg.Slot0.kA = Elevator.KA;
    fx_cfg.Slot0.kG = Elevator.KG;


    fx_cfg.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    SoftwareLimitSwitchConfigs limitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Elevator.MAX_SAFE_POS)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(2);

    fx_cfg.withSoftwareLimitSwitch(limitSwitchConfigs);

    fx_cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);

    elevatorMotor1.getConfigurator().apply(fx_cfg);


    setpoint = getEncoderPosition();

    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));


    // Log initial configuration
    Logger.recordOutput("Elevator/Config/MaxSafePos", Elevator.MAX_SAFE_POS);
    Logger.recordOutput("Elevator/Config/PIDEnabled", PIDEnabled);
    Logger.recordOutput("Elevator/Config/InitialSetpoint", setpoint);
  }

  public void setPosition(double position) {
    elevatorMotor1.setControl(positionVoltage.withPosition(position));
  }

  public double getEncoderPosition() {
    return elevatorMotor1.getRotorPosition().getValueAsDouble();
  }




  public void moveManual(double speed) {
    setClampSpeed(speed);
    Logger.recordOutput("Elevator/Command/ManualSpeed", speed);
  }

  private void setClampSpeed(double speed) {
    double clampedSpeed = Math.max(-1, Math.min(1, speed));
    elevatorMotor1.set(clampedSpeed);
    Logger.recordOutput("Elevator/Control/ClampSpeed", clampedSpeed);
  }

  public void stop() {
    elevatorMotor1.stopMotor();
    Logger.recordOutput("Elevator/Status", "Stopped");
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
    double position = elevatorMotor1.getPosition().getValueAsDouble();
    // double height = getHeight(position);
    double followPosition = elevatorMotor1.getPosition().getValueAsDouble();
    double followDifference = position - followPosition;
    double voltage = elevatorMotor1.getMotorVoltage().getValueAsDouble();

    // Log outputs using AdvantageKit
    Logger.recordOutput("Elevator/Status/Setpoint", setpoint);
    Logger.recordOutput("Elevator/Status/Position", getEncoderPosition());
    Logger.recordOutput(
        "Elevator/Status/Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Status/Current", elevatorMotor1.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Status/Temperature", elevatorMotor1.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Elevator/Control/PIDOutput", pidOutput);
    Logger.recordOutput("Elevator/Control/Feedforward", feedforward);
    Logger.recordOutput("Elevator/Control/TotalOutput", pidOutput + feedforward);
    Logger.recordOutput(
        "Elevator/Status/Velocity", elevatorMotor1.getRotorVelocity().getValueAsDouble());
  }
}
