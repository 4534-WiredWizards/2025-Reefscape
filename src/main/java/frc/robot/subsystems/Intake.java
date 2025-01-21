// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.C_Intake;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final SparkFlex intakeMotor = new SparkFlex(
    C_Intake.intakeMotorID,
    SparkLowLevel.MotorType.kBrushless
  );

  public Intake() {
    SparkFlexConfig baseConfig = new SparkFlexConfig();

    baseConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);

    intakeMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed) {
    intakeMotor.set(speed);
  }

  public void stopMotor() {
    intakeMotor.set(0);
  }

}
