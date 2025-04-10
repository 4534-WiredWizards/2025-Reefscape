// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AdaptiveWrist extends Command {
  private final IntakeSubsystem m_intake;
  private final DoubleSupplier wristAngleSupplier;
  private final boolean isPickup;

  // The separate coral intake command
  private RunCoralIntake coralIntakeCommand;
  private boolean isRunningCoralIntake = false;
  private boolean wasInCoralRange = false;

  // Protection delay timer
  private boolean protectionOverrideScheduled = false;
  private Timer protectionDelayTimer = new Timer();
  private static final double PROTECTION_DELAY_SECONDS = 1.0;

  public AdaptiveWrist(
      IntakeSubsystem intake, DoubleSupplier wristAngleSupplier, boolean isPickup) {
    this.m_intake = intake;
    this.wristAngleSupplier = wristAngleSupplier;
    this.isPickup = isPickup;

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    // Create a new instance of the coral intake command
    coralIntakeCommand = new RunCoralIntake(m_intake, true);
    m_intake.setProtectionOverride(true); // Disable protection during operation

    isRunningCoralIntake = false;
    wasInCoralRange = false;
    protectionOverrideScheduled = false;
    Logger.recordOutput("Wrist/Status/AdaptiveWrist", "Initialized");
  }

  @Override
  public void execute() {
    double wristAngle = wristAngleSupplier.getAsDouble();
    boolean isCoralRange = (wristAngle > Wrist.CORAL_MAX_ANGLE);

    // Check for mode changes
    if (isCoralRange != wasInCoralRange) {
      wasInCoralRange = isCoralRange;

      // If we're leaving coral range and were running coral intake, end it
      if (!isCoralRange && isRunningCoralIntake) {
        coralIntakeCommand.end(true);
        isRunningCoralIntake = false;
        Logger.recordOutput("Wrist/Status/AdaptiveWrist", "CoralIntake Ended");
      }
    }

    // Run appropriate intake/outtake based on mode
    if (isCoralRange) {
      if (isPickup) {
        // Handle coral pickup using the separate command
        handleCoralIntake();
        Logger.recordOutput("Wrist/Status/AdaptiveWrist", "CoralIntake Running");
      } else {
        // If switching from intake to outtake, end the coral intake command
        if (isRunningCoralIntake) {
          coralIntakeCommand.end(true);
          isRunningCoralIntake = false;
          Logger.recordOutput("Wrist/Status/AdaptiveWrist", "CoralIntake Ended");
        }
        // Outtake coral
        m_intake.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
        Logger.recordOutput("Wrist/Status/AdaptiveWrist", "Coral Outtake");
      }
    } else {
      // We're in algae range
      if (isPickup) {
        // Use the HoldAlgae command for algae pickup
        new HoldAlgae(m_intake).execute();
        // m_intake.moveRoller(Wrist.Roller.ALGAE_INTAKE_SPEED);
        Logger.recordOutput("Wrist/Status/AdaptiveWrist", "Algae Intake");
      } else {
        m_intake.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
        Logger.recordOutput("Wrist/Status/AdaptiveWrist", "Algae Outtake");
      }
    }
  }

  /** Manages the coral intake command lifecycle */
  private void handleCoralIntake() {
    // Start the coral intake command if not already running
    if (!isRunningCoralIntake) {
      coralIntakeCommand.initialize();
      isRunningCoralIntake = true;
    }

    // Run a single iteration of the coral intake command
    coralIntakeCommand.execute();

    // Check if coral intake is finished
    if (coralIntakeCommand.isFinished()) {
      // Successfully completed coral intake
      coralIntakeCommand.end(false);
      // Keep isRunningCoralIntake as true to prevent restarting the intake process
      // until we switch modes
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Make sure to properly end the coral intake command if it's running
    if (isRunningCoralIntake) {
      coralIntakeCommand.end(interrupted);
      isRunningCoralIntake = false;
    }

    // Instead of immediately re-enabling protection, start a timer
    // We'll handle this in the command scheduler through inheritance
    protectionDelayTimer.reset();
    protectionDelayTimer.start();
    protectionOverrideScheduled = true;

    // Log the delay for debug purposes
    Logger.recordOutput(
        "Wrist/Status/AdaptiveWrist",
        "Command ended - protection re-enable scheduled in "
            + PROTECTION_DELAY_SECONDS
            + " seconds");

    // Create a separate command to re-enable protection after the delay
    // This will run even after this command is no longer scheduled
    new EnableProtectionAfterDelayCommand(m_intake, protectionDelayTimer, PROTECTION_DELAY_SECONDS)
        .schedule();
  }

  /**
   * Inner class to handle re-enabling protection after delay This is a separate command that runs
   * independently
   */
  private static class EnableProtectionAfterDelayCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final Timer timer;
    private final double delaySeconds;

    public EnableProtectionAfterDelayCommand(
        IntakeSubsystem intake, Timer timer, double delaySeconds) {
      this.m_intake = intake;
      this.timer = timer;
      this.delaySeconds = delaySeconds;
      // No requirements - we want this to run in parallel with other commands
    }

    @Override
    public void execute() {
      // Just waiting for timer
      Logger.recordOutput("Intake/Protection/DelayRemaining", delaySeconds - timer.get());
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(delaySeconds);
    }

    @Override
    public void end(boolean interrupted) {
      // Re-enable protection when time elapses (or if interrupted)
      m_intake.setProtectionOverride(false);
      Logger.recordOutput("Intake/Protection/Status", "Protection re-enabled after delay");
    }
  }
}
