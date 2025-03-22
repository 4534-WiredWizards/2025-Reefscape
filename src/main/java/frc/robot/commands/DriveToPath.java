// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Command to pathfind to a prebuilt path and follow it */
public class DriveToPath extends Command {

  private final Drive drive;
  private final PathPlannerPath path;
  private Command pathFollowingCommand;
  private final PathConstraints constraints;

  /** Creates a new DriveToPoint with a prebuilt path and default constraints */
  public DriveToPath(Drive drive, PathPlannerPath path) {
    this.drive = drive;
    this.path = path;

    if (path == null) {
      throw new IllegalArgumentException("Path cannot be null");
    }

    // Create path constraints with default values
    this.constraints =
        new PathConstraints(
            drive.getMaxLinearSpeedMetersPerSec() * 0.5, // 50% of max velocity
            drive.getMaxLinearSpeedMetersPerSec() * 0.4, // 50% of max acceleration
            drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
            drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
            );

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPoint/StartPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/Status", "Pathfinding to start of prebuilt path");

    // Create pathfinding command to the prebuilt path
    pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

    // Schedule the path following command
    pathFollowingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update current status
    Logger.recordOutput("DriveToPoint/CurrentPose", drive.getPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathFollowingCommand != null) {
      pathFollowingCommand.cancel();
    }

    if (interrupted) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted");
    } else {
      Logger.recordOutput("DriveToPoint/Status", "Successfully followed path");
    }

    // Log final status
    Logger.recordOutput("DriveToPoint/FinalPose", drive.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFollowingCommand != null && pathFollowingCommand.isFinished();
  }
}
