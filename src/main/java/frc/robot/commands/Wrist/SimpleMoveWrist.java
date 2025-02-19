// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleMoveWrist extends Command {

  private final WristSubsystem m_wrist;
  private final DoubleSupplier speedSupplier;

  /** Creates a new SimpleMoveWrist. */
  public SimpleMoveWrist(WristSubsystem m_wrist, DoubleSupplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_wrist = m_wrist;
    this.speedSupplier = speedSupplier;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    speed = speed * Wrist.SPEED_SCALAR;
    Logger.recordOutput("Wrist/GivenSpeed", speed);


    m_wrist.moveManual(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.isStalled();
  }
}
