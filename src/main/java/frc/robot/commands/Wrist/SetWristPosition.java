// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.WristSubsystem;
import org.littletonrobotics.junction.Logger;

public class SetWristPosition extends Command {
  private final WristSubsystem m_wrist;
  private final double targetPosition;
  private final boolean stopWhenAtSetpoint;

  /** Creates a new SetWristPosition with stopWhenAtSetpoint defaulting to true. */
  public SetWristPosition(WristSubsystem m_wrist, double targetPosition) {
    this(m_wrist, targetPosition, true); // Default stopWhenAtSetpoint to true
  }

  /** Creates a new SetWristPosition with a specified stopWhenAtSetpoint value. */
  public SetWristPosition(
      WristSubsystem m_wrist, double targetPosition, boolean stopWhenAtSetpoint) {
    this.m_wrist = m_wrist;
    this.targetPosition = targetPosition;
    this.stopWhenAtSetpoint = stopWhenAtSetpoint;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetWristPosition initialized, target angle: " + targetPosition);
    double safePosition =
        Math.min(Wrist.MAX_SAFE_ANGLE, Math.max(Wrist.MIN_SAFE_ANGLE, targetPosition));
    m_wrist.setWristSetpoint(safePosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_wrist.getAngle();
    Logger.recordOutput(
        "Wrist/Command/Progress",
        (currentAngle - Wrist.MIN_SAFE_ANGLE) / (targetPosition - Wrist.MIN_SAFE_ANGLE) * 100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_wrist.stop();
      Logger.recordOutput("Wrist/Command/Interrupted", true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (stopWhenAtSetpoint && m_wrist.isAtPosition(targetPosition)) || m_wrist.isStalled();
  }
}
