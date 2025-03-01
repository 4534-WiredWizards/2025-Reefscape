// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPosition extends Command {
  private final ElevatorSubsystem m_elevator;
  private final double m_targetPosition;
  private final boolean stopWhenAtSetpoint;

  public SetElevatorPosition(ElevatorSubsystem elevator, double targetPosition) {
    this(elevator, targetPosition, true); // Default stopWhenAtSetpoint to true
  }

  public SetElevatorPosition(ElevatorSubsystem elevator, double targetPosition, boolean stopWhenAtSetpoint) {
    this.m_elevator = elevator;
    this.m_targetPosition = targetPosition;
    this.stopWhenAtSetpoint = stopWhenAtSetpoint;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    double safePosition =
        Math.min(Elevator.MAX_SAFE_POS, Math.max(Elevator.MIN_SAFE_POS, m_targetPosition));
    m_elevator.setPosition(safePosition);
  }

  @Override
  public void execute() {
    double currentPosition = m_elevator.getEncoderPosition();
    Logger.recordOutput(
        "Elevator/Command/Progress",
        (currentPosition - Elevator.MIN_SAFE_POS)
            / (m_targetPosition - Elevator.MIN_SAFE_POS)
            * 100);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_elevator.stop();
      Logger.recordOutput("Elevator/Command/Interrupted", true);
    }
  }

  @Override
  public boolean isFinished() {
    return (stopWhenAtSetpoint && m_elevator.isAtPosition(m_targetPosition)) || m_elevator.isStalled();
  }
}
