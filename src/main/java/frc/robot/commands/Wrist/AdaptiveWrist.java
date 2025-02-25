// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

public class AdaptiveWrist extends Command {
  private final IntakeSubsystem m_intake;
  private final DoubleSupplier wristAngleSupplier;
  private final boolean isPickup;
  
  // The separate coral intake command
  private RunCoralIntake coralIntakeCommand;
  private boolean isRunningCoralIntake = false;
  private boolean wasInCoralRange = false;
  
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
    isRunningCoralIntake = false;
    wasInCoralRange = false;
  }

  @Override
  public void execute() {
    double wristAngle = wristAngleSupplier.getAsDouble();
    boolean isCoralRange = wristAngle < Wrist.CORAL_MAX_ANGLE;
    
    // Check for mode changes
    if (isCoralRange != wasInCoralRange) {
      wasInCoralRange = isCoralRange;
      
      // If we're leaving coral range and were running coral intake, end it
      if (!isCoralRange && isRunningCoralIntake) {
        coralIntakeCommand.end(true);
        isRunningCoralIntake = false;
      }
    }

    // Run appropriate intake/outtake based on mode
    if (isCoralRange) {
      if (isPickup) {
        // Handle coral pickup using the separate command
        handleCoralIntake();
      } else {
        // If switching from intake to outtake, end the coral intake command
        if (isRunningCoralIntake) {
          coralIntakeCommand.end(true);
          isRunningCoralIntake = false;
        }
        // Outtake coral
        m_intake.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
      }
    } else {
      // We're in algae range
      if (isPickup) {
        m_intake.moveRoller(Wrist.Roller.ALGAE_INTAKE_SPEED);
      } else {
        m_intake.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
      }
    }
  }
  
  /**
   * Manages the coral intake command lifecycle
   */
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
  }
}