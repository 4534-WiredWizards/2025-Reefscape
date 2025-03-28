// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;

public class ClimbSubsystem extends SubsystemBase {
  // define climb motor
  public final SparkFlex climbMotor;
  // define reverse limit switch
  private final SparkLimitSwitch reverseLimitSwitch;

  /** Creates a new climb. */
  public ClimbSubsystem() {
    climbMotor = new SparkFlex(Climb.MOTOR_ID, SparkFlex.MotorType.kBrushless);

    // Initialize the reverse limit switch
    reverseLimitSwitch = climbMotor.getReverseLimitSwitch();

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

    // Configure limit switch - set to false so we can read it but it won't stop the motor
    motorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    Logger.recordOutput("Climb/Encoder", climbMotor.getEncoder().getPosition());

    climbMotor.configure(
        motorConfig,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Log position and limit switch state
    Logger.recordOutput("Climb/Encoder", climbMotor.getEncoder().getPosition());
    Logger.recordOutput("Climb/ReverseLimitSwitch", getReverseLimitSwitch());
  }

  public void moveManual(double speed) {
    climbMotor.set(speed);
  }

  public void stop() {
    climbMotor.set(0);
  }
  
  public boolean isStalled() {
    // Implement stall detection logic here if needed
    return false;
  }

  /**
   * Gets the state of the reverse limit switch
   *
   * @return true if the limit switch is pressed, false otherwise
   */
  public boolean getReverseLimitSwitch() {
    return reverseLimitSwitch.isPressed();
  }

  /**
   * Sets the idle mode for the climb motor
   *
   * @param mode The idle mode (kBrake or kCoast)
   */
  public void setIdleMode(IdleMode mode) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(mode);
    climbMotor.configure(
        config,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);
    Logger.recordOutput("Climb/IdleMode", mode.toString());
  }
}
