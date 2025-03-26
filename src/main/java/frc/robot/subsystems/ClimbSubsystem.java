// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;

public class ClimbSubsystem extends SubsystemBase {
  // define climb motor
  private final SparkFlex climbMotor;

  /** Creates a new climb. */
  public ClimbSubsystem() {
    climbMotor = new SparkFlex(Climb.MOTOR_ID, SparkFlex.MotorType.kBrushless);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
    Logger.recordOutput("Climb/Encoder", climbMotor.getEncoder().getPosition());

    climbMotor.configure(
        motorConfig,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Log position
    Logger.recordOutput("Climb/Encoder", climbMotor.getEncoder().getPosition());
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
   * Sets the idle mode for the climb motor
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