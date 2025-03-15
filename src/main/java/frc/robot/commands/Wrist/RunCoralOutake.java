// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

public class RunCoralOutake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final Timer commandTimer = new Timer();
  private static final double MINIMUM_RUN_TIME = 0.5; // Run for at least 0.5 seconds
  
  public RunCoralOutake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("RunCoralOutake initialized");
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
    commandTimer.reset();
    commandTimer.start();
    Logger.recordOutput("Intake/Command", "Coral Outake Started");
  }

  @Override
  public void execute() {
    // Continuously set roller speed in case of interruption
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunCoralOutake ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
    Logger.recordOutput("Intake/Command", "Coral Outake Ended");
  }

  @Override
  public boolean isFinished() {
    // Only check the sensor after the minimum run time has elapsed
    if (commandTimer.get() < MINIMUM_RUN_TIME) {
      return false;
    }
    
    // After the minimum time, stop when the coral is no longer detected
    boolean sensorDetection = !intakeSubsystem.getSecondSensor();
    
    if (sensorDetection) {
      System.out.println("RunCoralOutake finishing - second sensor no longer detects coral");
    }
    
    return sensorDetection;
  }
}