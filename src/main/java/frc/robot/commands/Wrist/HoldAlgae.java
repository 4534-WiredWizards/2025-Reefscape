package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * A command that runs the algae intake at reduced power to hold the algae in place until the
 * command is interrupted.
 */
public class HoldAlgae extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double holdPower;

  /**
   * Creates a new HoldAlgae command with the default power level.
   *
   * @param intakeSubsystem The intake subsystem
   */
  public HoldAlgae(IntakeSubsystem intakeSubsystem) {
    this(intakeSubsystem, 0.5); // Default to 50% of normal intake power
  }

  /**
   * Creates a new HoldAlgae command with a specified power level.
   *
   * @param intakeSubsystem The intake subsystem
   * @param powerPercentage Percentage of normal intake power (0.0 to 1.0)
   */
  public HoldAlgae(IntakeSubsystem intakeSubsystem, double powerPercentage) {
    this.intakeSubsystem = intakeSubsystem;
    this.holdPower =
        Wrist.Roller.ALGAE_INTAKE_SPEED * Math.max(0.0, Math.min(1.0, powerPercentage));
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("HoldAlgae initialized");
    intakeSubsystem.moveRoller(holdPower);
    Logger.recordOutput("Intake/Status/HoldingAlgae", true);
    Logger.recordOutput("Intake/Status/HoldPower", holdPower);
  }

  @Override
  public void execute() {
    // Keep applying the hold power
    intakeSubsystem.moveRoller(holdPower);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("HoldAlgae ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
    Logger.recordOutput("Intake/Status/HoldingAlgae", false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
