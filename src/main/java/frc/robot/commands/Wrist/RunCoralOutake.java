package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class RunCoralOutake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private boolean wasCoralDetected = false;

  public RunCoralOutake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("RunCoralOutake initialized");
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
    wasCoralDetected = intakeSubsystem.getSecondSensor();
    System.out.println("Initial coral detection: " + wasCoralDetected);
  }

  @Override
  public void execute() {
    System.out.println("RunCoralOutake executing");
    intakeSubsystem.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunCoralOutake ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
  }

  @Override
  public boolean isFinished() {
    boolean isFinished = wasCoralDetected && !intakeSubsystem.getFirstSensor();
    System.out.println("RunCoralOutake isFinished: " + isFinished);
    return isFinished;
  }
}
