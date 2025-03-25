// IntakeSubsystem.java
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex rollerMotor;
  // sensors
  private final SparkLimitSwitch firstSensor;
  private final SparkLimitSwitch secondSensor;

  public IntakeSubsystem() {
    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    firstSensor = rollerMotor.getForwardLimitSwitch();
    secondSensor = rollerMotor.getReverseLimitSwitch();

    SparkFlexConfig config = new SparkFlexConfig();
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.reverseLimitSwitchEnabled(false);

    // Increase current limit (carefully)
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(80);

    // Use the correct method for voltage compensation
    config.voltageCompensation(12.0);

    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Add ramping to avoid current spikes and maintain torque
  private double prevSpeed = 0;
  private final double RAMP_RATE = 0.1; // Adjust as needed

  public void moveRoller(double targetSpeed) {
    // Calculate new speed with ramping
    double newSpeed = prevSpeed;
    if (targetSpeed > prevSpeed) {
      newSpeed = Math.min(prevSpeed + RAMP_RATE, targetSpeed);
    } else if (targetSpeed < prevSpeed) {
      newSpeed = Math.max(prevSpeed - RAMP_RATE, targetSpeed);
    }

    rollerMotor.set(newSpeed);
    prevSpeed = newSpeed;
    Logger.recordOutput("Intake/Control/Speed", newSpeed);
  }

  public void stopRoller() {
    rollerMotor.set(0);
    // Logger.recordOutput("Intake/Control/Speed", 0);
  }

  public boolean getFirstSensor() {
    return firstSensor.isPressed();
  }

  public boolean getSecondSensor() {
    return secondSensor.isPressed();
  }

  // Periodic,
  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Sensor/First", getFirstSensor());
    Logger.recordOutput("Intake/Sensor/Second", getSecondSensor());
  }
}
