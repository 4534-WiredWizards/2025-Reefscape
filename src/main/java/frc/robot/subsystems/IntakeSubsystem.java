// IntakeSubsystem.java
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex rollerMotor;
  //sensors
  private final SparkLimitSwitch firstSensor;
  private final SparkLimitSwitch secondSensor;
  
  

  public IntakeSubsystem() {

    rollerMotor = new SparkFlex(Wrist.Roller.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    firstSensor =
      rollerMotor.getForwardLimitSwitch();
    secondSensor = 
      rollerMotor.getForwardLimitSwitch();
    
    
    
    
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
    return firstSensor.isPressed();
  }

  public boolean getSecondSensor() {
    return secondSensor.isPressed();
  }

}