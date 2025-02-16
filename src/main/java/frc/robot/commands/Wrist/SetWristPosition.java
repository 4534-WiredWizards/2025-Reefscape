// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

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

  // FIXME: Needs to contain a boolean to stop when finished or maintain PID control

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristSetpoint(targetPosition);
    m_wrist.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The PID controller will run in the subsystem's periodic method
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
    m_wrist.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopWhenAtSetpoint && m_wrist.atSetpoint();
  }
}
