// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {

  private final ElevatorSubsystem m_elevator;
  private final double targetPosition;

  public SetElevatorPosition(ElevatorSubsystem m_elevator, double targetPosition) {
    this.m_elevator = m_elevator;
    this.targetPosition = targetPosition;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    // Set the desired position
    m_elevator.setSetpoint(targetPosition);
    m_elevator.enable();
  }

  @Override
  public void execute() {
    // The PID controller will run in the subsystem's periodic method
  }

  @Override
  public void end(boolean interrupted) {
    // Check if the elevator has reached the target position (within a tolerance)
    m_elevator.stop();
    m_elevator.disable();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
