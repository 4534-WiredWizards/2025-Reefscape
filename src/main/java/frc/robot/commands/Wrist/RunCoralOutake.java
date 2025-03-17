// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class RunCoralOutake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private boolean initialSensorState;
  
  public RunCoralOutake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("RunCoralOutake initialized");
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
    initialSensorState = intakeSubsystem.getFirstSensor();
    Logger.recordOutput("Intake/Command", "Coral Outake Started");
    Logger.recordOutput("Intake/Outake/InitialSensorState", initialSensorState);
    System.out.println("RunCoralOutake - Initial first sensor state: " + 
                      (initialSensorState ? "Coral detected" : "No coral detected"));
  }

  @Override
  public void execute() {
    // Continuously set roller speed
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
    
    // Log current sensor state
    boolean currentSensorState = intakeSubsystem.getFirstSensor();
    Logger.recordOutput("Intake/Outake/CurrentSensorState", currentSensorState);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunCoralOutake ended, interrupted: " + interrupted);
    System.out.println("Final first sensor state: " + 
                      (intakeSubsystem.getFirstSensor() ? "Coral detected" : "No coral detected"));
    intakeSubsystem.stopRoller();
    Logger.recordOutput("Intake/Command", "Coral Outake Ended");
  }

  @Override
  public boolean isFinished() {
    // Finish when the first sensor no longer detects coral
    boolean sensorState = intakeSubsystem.getFirstSensor();
    
    // If sensor state changed from true to false, we're done
    if (initialSensorState && !sensorState) {
      System.out.println("RunCoralOutake finishing - first sensor no longer detects coral");
      return true;
    }
    
    return false;
  }
}