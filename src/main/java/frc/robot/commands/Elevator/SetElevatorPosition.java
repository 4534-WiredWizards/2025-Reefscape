// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SetElevatorPosition extends Command {
  private final ElevatorSubsystem m_elevator;
  private final DoubleSupplier m_targetPositionSupplier;
  private final boolean stopWhenAtSetpoint;
  private final WristSubsystem m_wrist;
  private double m_targetPosition;

  /**
   * Creates a SetElevatorPosition command with a fixed target position.
   *
   * @param elevator The elevator subsystem
   * @param targetPosition The fixed target position
   * @param wrist The wrist subsystem
   */
  public SetElevatorPosition(
      ElevatorSubsystem elevator, double targetPosition, WristSubsystem wrist) {
    this(elevator, targetPosition, wrist, true); // Default stopWhenAtSetpoint to true
  }

  /**
   * Creates a SetElevatorPosition command with a fixed target position.
   *
   * @param elevator The elevator subsystem
   * @param targetPosition The fixed target position
   * @param wrist The wrist subsystem
   * @param stopWhenAtSetpoint Whether to stop when the setpoint is reached
   */
  public SetElevatorPosition(
      ElevatorSubsystem elevator,
      double targetPosition,
      WristSubsystem wrist,
      boolean stopWhenAtSetpoint) {
    this(elevator, () -> targetPosition, wrist, stopWhenAtSetpoint);
  }

  /**
   * Creates a SetElevatorPosition command with a dynamic target position supplier.
   *
   * @param elevator The elevator subsystem
   * @param targetPositionSupplier Supplier for the target position
   * @param wrist The wrist subsystem
   */
  public SetElevatorPosition(
      ElevatorSubsystem elevator, DoubleSupplier targetPositionSupplier, WristSubsystem wrist) {
    this(elevator, targetPositionSupplier, wrist, true); // Default stopWhenAtSetpoint to true
  }

  /**
   * Creates a SetElevatorPosition command with a dynamic target position supplier.
   *
   * @param elevator The elevator subsystem
   * @param targetPositionSupplier Supplier for the target position
   * @param wrist The wrist subsystem
   * @param stopWhenAtSetpoint Whether to stop when the setpoint is reached
   */
  public SetElevatorPosition(
      ElevatorSubsystem elevator,
      DoubleSupplier targetPositionSupplier,
      WristSubsystem wrist,
      boolean stopWhenAtSetpoint) {
    this.m_elevator = elevator;
    this.m_targetPositionSupplier = targetPositionSupplier;
    this.stopWhenAtSetpoint = stopWhenAtSetpoint;
    this.m_wrist = wrist;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    // Get the current target position from the supplier at initialization time
    m_targetPosition = m_targetPositionSupplier.getAsDouble();

    double safePosition =
        Math.min(Elevator.MAX_SAFE_POS, Math.max(Elevator.MIN_SAFE_POS, m_targetPosition));

    if (m_targetPosition == Elevator.POSITION_GROUND) {
      m_elevator.setLookForStalled(true);
    } else {
      m_elevator.setLookForStalled(false);
    }

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
    } else if (m_targetPosition == Elevator.POSITION_GROUND) {
      m_elevator.setZeroCommand().schedule();
    }

    m_elevator.setLookForStalled(false);
  }

  @Override
  public boolean isFinished() {
    if (m_targetPosition == Elevator.POSITION_GROUND) {
      return (stopWhenAtSetpoint && m_elevator.isAtPosition(m_targetPosition) || ( m_elevator.isStalled() && m_elevator.getEncoderPosition()<Elevator.ELEVATOR_DANGER_LIMIT) );
    } else {
      return (stopWhenAtSetpoint && m_elevator.isAtPosition(m_targetPosition));
    }
  }
}
