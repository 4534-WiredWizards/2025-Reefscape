// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.C_Wrist;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class WristSubystem extends SubsystemBase {
  /** Creates a new Wrist. */
  private final SparkFlex wristMotor;
  private DutyCycleEncoder absEncoder;

  public WristSubsystem() {
    wristMotor = new SparkFlex(C_Wrist.wristMotorID, SparkLowLevel.MotorType.kBrushless);
    absEncoder = new DutyCycleEncoder(1);
    SparkFlexConfig baseConfig = new SparkFlexConfig();

    baseConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    wristMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double speed) {
    wristMotor.set(speed);
  }

  public void moveToPosition(double position) {
    // TO DO 
  }

  public double getAngle() {
    return absEncoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
