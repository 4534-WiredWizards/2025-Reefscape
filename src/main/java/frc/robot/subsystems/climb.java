// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climb extends SubsystemBase {

  // define climb motor
  private final SparkFlex climbMotor1;
  private final SparkFlex climbMotor2;
  /** Creates a new climb. */
  public climb() {

    climbMotor1 = new SparkFlex(56, SparkFlex.MotorType.kBrushless);
    climbMotor2 = new SparkFlex(57, SparkFlex.MotorType.kBrushless);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    SparkFlexConfig followConfig = new SparkFlexConfig();

    motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

    followConfig.follow(climbMotor1, true);

    climbMotor1.configure(
        motorConfig,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);

    climbMotor2.configure(
        followConfig,
        SparkFlex.ResetMode.kNoResetSafeParameters,
        SparkFlex.PersistMode.kNoPersistParameters);
  }
  ;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    climbMotor1.set(speed);
  }

  public void stop() {
    climbMotor1.set(0);
  }
}
