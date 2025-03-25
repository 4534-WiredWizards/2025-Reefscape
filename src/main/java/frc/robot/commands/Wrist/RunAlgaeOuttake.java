package frc.robot.commands.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class RunAlgaeOuttake extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double timeout;
  private long startTime;

  /**
   * Creates a new RunAlgaeOuttake command with the default timeout.
   *
   * @param intakeSubsystem The intake subsystem
   */
  public RunAlgaeOuttake(IntakeSubsystem intakeSubsystem) {
    this(intakeSubsystem, 1.5); // Default 1.5 second timeout
  }

  /**
   * Creates a new RunAlgaeOuttake command with a specified timeout.
   *
   * @param intakeSubsystem The intake subsystem
   * @param timeoutSeconds Timeout in seconds
   */
  public RunAlgaeOuttake(IntakeSubsystem intakeSubsystem, double timeoutSeconds) {
    this.intakeSubsystem = intakeSubsystem;
    this.timeout = timeoutSeconds * 1000; // Convert to milliseconds
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("RunAlgaeOuttake initialized");
    intakeSubsystem.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
    startTime = System.currentTimeMillis();
    Logger.recordOutput("Intake/Status/AlgaeOuttakeStarted", true);
  }

  @Override
  public void execute() {
    intakeSubsystem.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
    Logger.recordOutput("Intake/Status/AlgaeOuttakeRunning", true);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("RunAlgaeOuttake ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
    Logger.recordOutput("Intake/Status/AlgaeOuttakeRunning", false);
    Logger.recordOutput("Intake/Status/AlgaeOuttakeEnded", true);
  }

  @Override
  public boolean isFinished() {
    // Since algae doesn't affect sensors, we only use the timeout to determine completion
    boolean timedOut = System.currentTimeMillis() - startTime > timeout;
    
    if (timedOut) {
      System.out.println("RunAlgaeOuttake completed after timeout: " + timeout + "ms");
    }
    
    return timedOut;
  }
}