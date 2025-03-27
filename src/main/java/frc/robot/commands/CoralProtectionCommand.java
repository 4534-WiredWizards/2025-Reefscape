// CoralProtectionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

public class CoralProtectionCommand extends Command {
  private final IntakeSubsystem m_intake;

  // State tracking
  private boolean wasSecondSensorActive = false;
  private boolean overrideActive = false;
  private boolean isRecovering = false;
  private int recoveryDurationCounter = 0;
  private static final int RECOVERY_DURATION = 15; // ~0.3 seconds at 50Hz execution

  public CoralProtectionCommand(IntakeSubsystem intake) {
    this.m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    wasSecondSensorActive = m_intake.getSecondSensor();
    overrideActive = false;
    isRecovering = false;
    recoveryDurationCounter = 0;
    Logger.recordOutput("Intake/Protection/Status", "Initialized");
  }

  @Override
  public void execute() {
    // If override is active, do nothing
    if (overrideActive) {
      return;
    }

    // Get current sensor states
    boolean firstSensorActive = m_intake.getFirstSensor();
    boolean secondSensorActive = m_intake.getSecondSensor();

    // Check for the specific slippage condition:
    // 1. First sensor is active (we have a coral)
    // 2. Second sensor was active before but now it's not (coral slipped)
    // 3. We're not currently in override mode
    if (firstSensorActive && wasSecondSensorActive && !secondSensorActive && !isRecovering) {
      // Start recovery!
      isRecovering = true;
      recoveryDurationCounter = 0;
      Logger.recordOutput("Intake/Protection/Status", "Coral slipping - activating recovery");
    }

    // If we're in recovery mode
    if (isRecovering) {
      // Apply recovery force
      m_intake.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED * 0.8);
      recoveryDurationCounter++;

      // End recovery after a set duration or if second sensor is active again
      if (recoveryDurationCounter >= RECOVERY_DURATION || secondSensorActive) {
        isRecovering = false;
        m_intake.stopRoller();
        Logger.recordOutput(
            "Intake/Protection/Status",
            secondSensorActive ? "Recovery succeeded" : "Recovery timeout");
      }
    }

    // Update previous state for next cycle
    wasSecondSensorActive = secondSensorActive;

    // Log current state for debugging
    Logger.recordOutput("Intake/Protection/FirstSensor", firstSensorActive);
    Logger.recordOutput("Intake/Protection/SecondSensor", secondSensorActive);
    Logger.recordOutput("Intake/Protection/IsRecovering", isRecovering);
    Logger.recordOutput("Intake/Protection/OverrideActive", overrideActive);
  }

  @Override
  public void end(boolean interrupted) {
    if (isRecovering) {
      m_intake.stopRoller();
      isRecovering = false;
    }
    Logger.recordOutput(
        "Intake/Protection/Status", interrupted ? "Command interrupted" : "Command ended");
  }

  /** Call this method to temporarily disable the protection system during outtake */
  public void setOverrideActive(boolean override) {
    this.overrideActive = override;

    // If turning on override while recovering, stop recovery
    if (override && isRecovering) {
      isRecovering = false;
      m_intake.stopRoller();
    }

    Logger.recordOutput(
        "Intake/Protection/Status", override ? "Override active" : "Override inactive");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
