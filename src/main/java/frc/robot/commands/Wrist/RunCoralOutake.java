// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class RunCoralOutake extends Command {
  private final IntakeSubsystem intakeSubsystem;

  public RunCoralOutake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
  }

  @Override
  public void execute() {
    // Continuously set roller speed in case of interruption
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopRoller();
  }

  @Override
  public boolean isFinished() {
    // Stop command when the coral is no longer detected
    return !intakeSubsystem.getSecondSensor();
  }
}