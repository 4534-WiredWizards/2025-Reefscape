package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Create a private method to initialize constraints
  private PathConstraints createDefaultConstraints(Drive drive) {
    return new PathConstraints(
        drive.getMaxLinearSpeedMetersPerSec() * 0.6, // 100% of max velocity
        drive.getMaxLinearSpeedMetersPerSec() * 0.4, // 200% of max acceleration
        drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
        drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
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

  // Modify the initialize method
  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPoint/StartPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/Status", "Pathfinding to start of prebuilt path");

    // Start auto-align LED sequence
    RobotContainer.LEDSubsystem.autoAlignMode(true);

    // Reset interrupt tracking
    wasInterruptedByTrigger = false;

    // Create pathfinding command to the prebuilt path
    pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints).until(interrupter);

    // Schedule the path following command
    pathFollowingCommand.schedule();
  }

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

  // Modify the end method
  @Override
  public void end(boolean interrupted) {
    // Stop auto-align LED sequence
    if (interrupted || wasInterruptedByTrigger) {
      RobotContainer.LEDSubsystem.showError();
    } else {
      RobotContainer.LEDSubsystem.showIntakeSuccess();
    }

    if (pathFollowingCommand != null) {
      pathFollowingCommand.cancel();
    }

    if (interrupted || wasInterruptedByTrigger) {
      Logger.recordOutput("DrivPeToPoint/Status", "Command interrupted");
    } else {
      Logger.recordOutput("DriveToPoint/Status", "Successfully followed path");
    }

    // Log final status
    Logger.recordOutput("DriveToPoint/FinalPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/WasInterruptedByTrigger", wasInterruptedByTrigger);

    // Make sure we stop the drive when the command ends
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    if (interrupter != null && interrupter.getAsBoolean()) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted by trigger");
      return true;
    }

    return pathFollowingCommand != null && pathFollowingCommand.isFinished();
  }
}
