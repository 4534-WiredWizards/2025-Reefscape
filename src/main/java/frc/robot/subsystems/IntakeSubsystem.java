// IntakeSubsystem.java
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex rollerMotor;
  // sensors
  private final DigitalInput firstSensor;
  private final DigitalInput secondSensor;

  public IntakeSubsystem() {
    // Intake config

    firstSensor = new DigitalInput(Wrist.Roller.FIRST_SENSOR_ID);
    secondSensor = new DigitalInput(Wrist.Roller.SECOND_SENSOR_ID);

    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkFlexConfig idleConfig = new SparkFlexConfig();
    idleConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    rollerMotor.configure(
        idleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveRoller(double speed) {
    rollerMotor.set(speed);
    Logger.recordOutput("Intake/Control/Speed", speed);
  }

  public void stopRoller() {
    rollerMotor.set(0);
    Logger.recordOutput("Intake/Control/Speed", 0);
  }

  public boolean getFirstSensor() {
    return firstSensor.get();
  }

  public boolean getSecondSensor() {
    return secondSensor.get();
  }
}
