package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/** Command to pathfind to a prebuilt path and follow it */
public class DriveToPath extends Command {

  private final Drive drive;
  private final PathPlannerPath path;
  private Command pathFollowingCommand;
  private final PathConstraints constraints;
  private final BooleanSupplier interrupter;

  // Track if interrupted by trigger
  private boolean wasInterruptedByTrigger = false;

  /** Creates a new DriveToPoint with a prebuilt path and default constraints */
  public DriveToPath(Drive drive, PathPlannerPath path, BooleanSupplier interrupter) {
    this.drive = drive;
    this.path = path;
    this.interrupter = interrupter;

    if (path == null) {
      throw new IllegalArgumentException("Path cannot be null");
    }

    // Create path constraints with default values
    this.constraints =
        new PathConstraints(
          drive.getMaxLinearSpeedMetersPerSec(), // 100% of max velocity
          drive.getMaxLinearSpeedMetersPerSec() * 2, // 200% of max acceleration
          drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
          drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
            );

    addRequirements(drive);
  }

  /** Creates a new DriveToPoint with a prebuilt path and default constraints */
  public DriveToPath(Drive drive, PathPlannerPath path) {
    this.drive = drive;
    this.path = path;
    this.interrupter = null;

    if (path == null) {
      throw new IllegalArgumentException("Path cannot be null");
    }

    // Create path constraints with default values
    this.constraints =
        new PathConstraints(
            drive.getMaxLinearSpeedMetersPerSec(), // 100% of max velocity
            drive.getMaxLinearSpeedMetersPerSec() * 2, // 200% of max acceleration
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
    
    // Reset interrupt tracking
    wasInterruptedByTrigger = false;

    // Create pathfinding command to the prebuilt path
    pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints).until(interrupter);

    // Schedule the path following command
    pathFollowingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update current status
    Logger.recordOutput("DriveToPoint/CurrentPose", drive.getPose());

    // Check if interrupter is active
    if (interrupter != null) {
      boolean currentInterruptState = interrupter.getAsBoolean();
      Logger.recordOutput("DriveToPoint/InterrupterActive", currentInterruptState);

      // Record if we were interrupted by trigger for later use
      if (currentInterruptState) {
        wasInterruptedByTrigger = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathFollowingCommand != null) {
      pathFollowingCommand.cancel();
    }

    if (interrupted || wasInterruptedByTrigger) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted");
    } else {
      Logger.recordOutput("DriveToPoint/Status", "Successfully followed path");
    }
    // Log final status
    Logger.recordOutput("DriveToPoint/FinalPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/WasInterruptedByTrigger", wasInterruptedByTrigger);

    // Make sure we stop the drive when the command ends
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (interrupter != null && interrupter.getAsBoolean()) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted by trigger");
      return true;
    }

    return pathFollowingCommand != null && pathFollowingCommand.isFinished();
  }
}