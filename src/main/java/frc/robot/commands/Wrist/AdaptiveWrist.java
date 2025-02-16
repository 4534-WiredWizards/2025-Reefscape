// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdaptiveWrist extends Command {

  public final WristSubsystem m_wrist;
  private final boolean isPickup;

  /** Creates a new AdaptiveWrist. */
  public AdaptiveWrist(WristSubsystem m_wrist, boolean isPickup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_wrist = m_wrist;
    this.isPickup = isPickup;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // FIXME: Needs to contain way to check when finished
  // Will be done with sensors attached to spark flex

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double wristAngle = m_wrist.getAngle();

    boolean isCoralRange = wristAngle > Wrist.CORAL_MIN_ANGLE && wristAngle < Wrist.CORAL_MAX_ANGLE;

    if (isPickup) {
      if (isCoralRange) {
        m_wrist.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED);
      } else {
        m_wrist.moveRoller(Wrist.Roller.CORAL_OUTTAKE_SPEED);
      }
    } else {
      if (isCoralRange) {
        m_wrist.moveRoller(Wrist.Roller.ALGAE_INTAKE_SPEED);
      } else {
        m_wrist.moveRoller(Wrist.Roller.ALGAE_OUTTAKE_SPEED);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
