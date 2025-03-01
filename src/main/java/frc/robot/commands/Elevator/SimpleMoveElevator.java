// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleMoveElevator extends Command {

  private final ElevatorSubsystem m_elevator;
  private final DoubleSupplier speedSupplier;
  private final WristSubsystem m_Wrist;

  public SimpleMoveElevator(
      WristSubsystem wrist, ElevatorSubsystem m_elevator, DoubleSupplier speedSupplier) {
    this.m_elevator = m_elevator;
    this.speedSupplier = speedSupplier;
    this.m_Wrist = wrist;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    // Stop the PID controller when manual control starts
    m_elevator.Stop();
  }

  @Override
  public void execute() {
    // Move the elevator manually based on the input speed
    double speed = speedSupplier.getAsDouble();
    m_elevator.moveManual(speed);
    // if (m_Wrist.getAngle() > Wrist.MIN_CLEAR_ELEVATOR_ANGLE) {
    //   m_Wrist.moveManual(-.2);
    // } else {

    //
    // }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends

    m_elevator.Stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously until interrupted
    return m_elevator.isStalled();
  }
}
