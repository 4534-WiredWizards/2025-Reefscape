// CoralProtectionCommand.java
package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class CoralProtectionCommand extends Command {
    private final IntakeSubsystem m_intake;
    
    // State tracking
    private boolean coralDetected = false;
    private boolean overrideActive = false;
    private int retryCount = 0;
    private static final int MAX_RETRIES = 3;
    
    public CoralProtectionCommand(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        coralDetected = m_intake.getSecondSensor();
        overrideActive = false;
        retryCount = 0;
        Logger.recordOutput("Intake/Protection/Status", "Initialized");
    }
    
    @Override
    public void execute() {
        boolean currentCoralSensor = m_intake.getSecondSensor();
        boolean firstSensor = m_intake.getFirstSensor();
        
        // Update state tracking
        if (!coralDetected && currentCoralSensor) {
            // Coral has been newly detected by the second sensor
            coralDetected = true;
            Logger.recordOutput("Intake/Protection/Status", "Coral detected");
        } else if (coralDetected && !currentCoralSensor && !overrideActive) {
            // Coral was detected but has slipped (not during an override)
            Logger.recordOutput("Intake/Protection/Status", "Coral slipping - recovering");
            
            // Only attempt recovery if we haven't exceeded max retries
            if (retryCount < MAX_RETRIES) {
                // Apply intake force to pull coral back in
                m_intake.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED * 0.7);
                retryCount++;
            } else {
                // We've tried several times but can't seem to recover the coral
                Logger.recordOutput("Intake/Protection/Status", "Max retries reached");
                coralDetected = false;
                retryCount = 0;
            }
        } else if (currentCoralSensor) {
            // Coral is still detected, reset retry count
            retryCount = 0;
            // If no command is controlling the intake, maintain a very small holding force
            if (!overrideActive) {
                m_intake.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED * 0.1);
            }
        }
        
        // Update coral detection state for next cycle
        coralDetected = currentCoralSensor;
        
        // Log current state for debugging
        Logger.recordOutput("Intake/Protection/CoralDetected", coralDetected);
        Logger.recordOutput("Intake/Protection/RetryCount", retryCount);
        Logger.recordOutput("Intake/Protection/OverrideActive", overrideActive);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Only stop the motor if we're ending for real, not just being preempted temporarily
        if (interrupted) {
            Logger.recordOutput("Intake/Protection/Status", "Command interrupted");
        } else {
            m_intake.stopRoller();
            Logger.recordOutput("Intake/Protection/Status", "Command ended");
        }
    }
    
    /**
     * Call this method to temporarily disable the protection system during outtake
     */
    public void setOverrideActive(boolean override) {
        this.overrideActive = override;
        if (override) {
            Logger.recordOutput("Intake/Protection/Status", "Override active");
        } else {
            Logger.recordOutput("Intake/Protection/Status", "Override inactive");
        }
    }
    
    /**
     * Override isFinished to make this a non-ending command
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}