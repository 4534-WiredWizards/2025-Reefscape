// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoint extends Command {

  private final Drive drive;
  private final Pose2d targetPose;
  private Command pathFindingCommand;

  /** Creates a new DriveToPoint. */
  public DriveToPoint(Drive drive, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.targetPose = targetPose;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    PathConstraints constraints =
        new PathConstraints(
            drive.getMaxLinearSpeedMetersPerSec() / 8, // Maximum linear velocity (m/s)
            drive.getMaxLinearSpeedMetersPerSec() / 2, // Maximum linear acceleration (m/s²)
            drive.getMaxAngularSpeedRadPerSec(), // Maximum angular velocity (rad/s)
            drive.getMaxAngularSpeedRadPerSec() // Maximum angular acceleration (rad/s²)
            );
    pathFindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    pathFindingCommand.schedule();
    System.out.println("DriveToPoint initialized!!");
    // Log the target pose
    System.out.println("Target Pose: " + targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathFindingCommand.cancel();
    // End log
    System.out.println("DriveToPoint ended!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFindingCommand.isFinished();
  }
}
