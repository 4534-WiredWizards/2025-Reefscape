package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class RunCoralOutake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private boolean wasCoralDetected = false;
  private final double outakeSpeed;

  /**
   * Creates a command to outake a coral game piece.
   *
   * @param intakeSubsystem The intake subsystem
   * @param outakeSpeed The speed to outake at (optional, defaults to CORAL_OUTTAKE_SPEED)
   */
  public RunCoralOutake(IntakeSubsystem intakeSubsystem, double outakeSpeed) {
    this.intakeSubsystem = intakeSubsystem;
    this.outakeSpeed = outakeSpeed;
    addRequirements(intakeSubsystem);
  }

  /**
   * Creates a command to outake a coral game piece at the default speed.
   *
   * @param intakeSubsystem The intake subsystem
   */
  public RunCoralOutake(IntakeSubsystem intakeSubsystem) {
    this(intakeSubsystem, Wrist.Roller.CORAL_OUTTAKE_SPEED);
  }

  @Override
  public void initialize() {
    System.out.println("RunCoralOutake initialized");
    intakeSubsystem.setProtectionOverride(true); // Disable protection during outake
    intakeSubsystem.moveRoller(outakeSpeed);
    wasCoralDetected = intakeSubsystem.getSecondSensor();
    System.out.println("Initial coral detection: " + wasCoralDetected);
  }

  @Override
  public void execute() {
    System.out.println("RunCoralOutake executing");
    intakeSubsystem.moveRoller(outakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunCoralOutake ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
    intakeSubsystem.setProtectionOverride(false); // Re-enable protection
  }

  @Override
  public boolean isFinished() {
    boolean isFinished = wasCoralDetected && !intakeSubsystem.getFirstSensor();
    System.out.println("RunCoralOutake isFinished: " + isFinished);
    return isFinished;
  }
}
