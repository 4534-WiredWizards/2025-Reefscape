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
import frc.robot.commands.CoralProtectionCommand;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex rollerMotor;
  // sensors
  private final SparkLimitSwitch firstSensor;
  private final SparkLimitSwitch secondSensor;
  private CoralProtectionCommand protectionCommand = null;

  // Add ramping to avoid current spikes and maintain torque
  private double prevSpeed = 0;
  private final double RAMP_RATE = 0.1; // Adjust as needed

  public IntakeSubsystem() {
    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    firstSensor = rollerMotor.getForwardLimitSwitch();
    secondSensor = rollerMotor.getReverseLimitSwitch();

    SparkFlexConfig config = new SparkFlexConfig();
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.reverseLimitSwitchEnabled(false);

    // Increase current limit (carefully)
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(
            80, 40); // Increase current limits for more torque while limiting stall current

    // Use the correct method for voltage compensation
    config.voltageCompensation(15.0);

    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

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

  /**
   * Sets a specific current/torque level for the roller motor. This provides more consistent force
   * regardless of position.
   *
   * @param currentAmps Target current in amps
   */
  public void setTorque(double currentAmps) {
    // Implement torque control via duty cycle approximation
    // This is a simplified approach since SparkFlex doesn't have direct current control

    // Clamp to safe range
    double safeCurrent = Math.max(0.0, Math.min(currentAmps, 40.0)); // Max 40 amps for safety

    // Convert current to approximate duty cycle
    // This is an approximation that may need tuning for your specific motor
    double estimatedDutyCycle = safeCurrent / 80.0; // 80A is roughly max stall current

    // Apply with ramping for safety
    moveRoller(estimatedDutyCycle);

    Logger.recordOutput("Intake/Control/TargetCurrent", safeCurrent);
    Logger.recordOutput("Intake/Control/EstimatedDutyCycle", estimatedDutyCycle);
  }

  public void stopRoller() {
    rollerMotor.set(0);
    prevSpeed = 0;
    Logger.recordOutput("Intake/Control/Speed", 0);
  }

  public boolean getFirstSensor() {
    return firstSensor.isPressed();
  }

  public boolean getSecondSensor() {
    return secondSensor.isPressed();
  }

  /** Gets the current motor current in amps */
  public double getCurrentAmps() {
    return rollerMotor.getOutputCurrent();
  }

  // Add this method to the IntakeSubsystem class
  public CoralProtectionCommand getProtectionCommand() {
    if (protectionCommand == null) {
      protectionCommand = new CoralProtectionCommand(this);
    }
    return protectionCommand;
  }

  // Add this method to the IntakeSubsystem class
  public void setProtectionOverride(boolean override) {
    if (protectionCommand != null) {
      protectionCommand.setOverrideActive(override);
    }
  }

  // Periodic
  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Sensor/First", getFirstSensor());
    Logger.recordOutput("Intake/Sensor/Second", getSecondSensor());
    Logger.recordOutput("Intake/Status/Current", getCurrentAmps());
  }
}
