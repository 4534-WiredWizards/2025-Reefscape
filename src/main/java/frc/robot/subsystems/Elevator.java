// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class Elevator extends SubsystemBase {

  // Define Limitswitches and neo vortex motor
  private final SparkLimitSwitch bottomLimitSwitch;
  private final SparkFlex leftDrive = new SparkFlex(
      Elevator.leftMotorID,
      SparkLowLevel.MotorType.kBrushless);
  private final SparkFlex rightDrive = new SparkFlex(
      Elevator.rightMotorID,
      SparkLowLevel.MotorType.kBrushless);
private final RelativeEncoder leftEncoder = leftDrive.getEncoder();

  /** Creates a new Elevator. */
  public Elevator() {
    SparkFlexConfig baseConfig = new SparkFlexConfig();
    SparkFlexConfig followConfig = new SparkFlexConfig();
    IntakeLimitSwitch = 

    baseConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    followConfig
        .follow(leftDrive, true);

    leftDrive.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDrive.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDrive.configure(followConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    leftDrive.set(speed);
  }

  public double getSpeed() {
    return leftEncoder.getVelocity();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
