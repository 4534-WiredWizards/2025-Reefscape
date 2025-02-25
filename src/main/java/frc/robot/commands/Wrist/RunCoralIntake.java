// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralIntake extends Command {
  /** Creates a new RunIntake. */
  private IntakeSubsystem intakeSubsystem;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem = new IntakeSubsystem();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public void runIntake() {
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED);
    if (intakeSubsystem.getFirstSensor()) {
      intakeSubsystem.moveRoller(Wrist.Roller.AFTER_FIRST_SENSOR_CORAL_SPEED);
    }
    if (intakeSubsystem.getSecondSensor()) {
      new WaitCommand(1);
      intakeSubsystem.stopRoller();
    }

    // If first sensor no longer sees the coral,  roll back until it does

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
