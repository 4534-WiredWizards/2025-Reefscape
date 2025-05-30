package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Command to pathfind to a prebuilt path and follow it */
public class DriveToPath extends Command {

  private final Drive drive;
  private final PathPlannerPath path;
  private Command pathFollowingCommand;
  private final PathConstraints constraints;
  private final BooleanSupplier interrupter;

  // Track if interrupted by trigger
  private boolean wasInterruptedByTrigger = false;
  private boolean hasStarted = false;

  // Create a private method to initialize constraints

  // Older faster driving
  // private PathConstraints createDefaultConstraints(Drive drive) {
  //   return new PathConstraints(
  //       drive.getMaxLinearSpeedMetersPerSec() * 0.6, // 100% of max velocity
  //       drive.getMaxLinearSpeedMetersPerSec() * 0.4, // 200% of max acceleration
  //       drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
  //       drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
  //       );
  // }

  private PathConstraints createDefaultConstraints(Drive drive) {
    return new PathConstraints(
        drive.getMaxLinearSpeedMetersPerSec() * 0.3, // Reduce linear speed to 30% of max velocity
        drive.getMaxLinearSpeedMetersPerSec()
            * 0.2, // Reduce acceleration to 20% of max acceleration
        drive.getMaxAngularSpeedRadPerSec()
            * 0.5, // Reduce angular velocity to 50% of max angular velocity
        drive.getMaxAngularSpeedRadPerSec()
            * 0.5 // Reduce angular acceleration to 50% of max angular acceleration
        );
  }

  /** Creates a new DriveToPoint with a prebuilt path and default constraints */
  public DriveToPath(Drive drive, PathPlannerPath path, BooleanSupplier interrupter) {
    this.drive = drive;
    this.path = path;
    this.interrupter = interrupter;

    if (path == null) {
      throw new IllegalArgumentException("Path cannot be null");
    }

    // Use the helper method to create constraints
    this.constraints = createDefaultConstraints(drive);

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

    // Use the same helper method for constraints
    this.constraints = createDefaultConstraints(drive);

    addRequirements(drive);
  }

  // Rest of the code remains unchanged...
  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPoint/StartPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/Status", "Pathfinding to start of prebuilt path");

    Command existingCommand = CommandScheduler.getInstance().requiring(drive);
    if (existingCommand != null && existingCommand != this) {
      Logger.recordOutput(
          "DriveToPoint/Status", "Command skipped - another path is already running");
      hasStarted = false;
      return; // Skip initialization
    }

    hasStarted = true;

    // Reset interrupt tracking
    wasInterruptedByTrigger = false;
    RobotContainer.setAutoDriving(true);
    // Create pathfinding command to the prebuilt path
    pathFollowingCommand =
        AutoBuilder.pathfindThenFollowPath(path, constraints)
            .until(interrupter != null ? interrupter : () -> false)
            .andThen(new InstantCommand(() -> RobotContainer.setAutoDriving(false)));

    // Schedule the path following command
    pathFollowingCommand.schedule();
  }

  @Override
  public void execute() {

    if (!hasStarted) {
      return;
    }
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

  @Override
  public void end(boolean interrupted) {

    if (hasStarted) {
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
  }

  @Override
  public boolean isFinished() {

    if (!hasStarted) {
      return true;
    }

    if (interrupter != null && interrupter.getAsBoolean()) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted by trigger");
      return true;
    }

    return pathFollowingCommand != null && pathFollowingCommand.isFinished();
  }
}
