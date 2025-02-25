// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdaptiveWrist extends Command {
  private final IntakeSubsystem m_intake;
  private final DoubleSupplier wristAngleSupplier;
  private final boolean isPickup;
  private final RunCoralIntake coralIntake;
  private double speed;

  public AdaptiveWrist(IntakeSubsystem intake, DoubleSupplier wristAngleSupplier, boolean isPickup) {
    this.m_intake = intake;
    this.wristAngleSupplier = wristAngleSupplier;
    this.isPickup = isPickup;

    coralIntake = new RunCoralIntake();

    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    double wristAngle = wristAngleSupplier.getAsDouble();
    boolean isCoralRange = wristAngle < Wrist.CORAL_MAX_ANGLE;
    
    runIntake(isCoralRange);
  }

  private void runIntake(boolean isCoralRange) {
    if (isCoralRange) {
        if (isPickup) {
            coralIntake.runIntake();
        } else {
            m_intake.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
        }
    } else {
        if (isPickup) {
            m_intake.moveRoller(Wrist.Roller.ALGAE_INTAKE_SPEED);
        } else {
            m_intake.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
        }
    }
}
}