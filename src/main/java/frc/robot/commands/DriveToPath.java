package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Debug variables
  private boolean wasInterruptedByTrigger = false;
  private long lastDebugPrintTime = 0;
  private static final long DEBUG_PRINT_INTERVAL_MS = 500; // Print debug every 500ms

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
            drive.getMaxLinearSpeedMetersPerSec() * 0.5, // 50% of max velocity
            drive.getMaxLinearSpeedMetersPerSec() * 0.4, // 40% of max acceleration
            drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
            drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
            );

    addRequirements(drive);
    System.out.println(
        "[DriveToPath] Constructor with interrupter called for path: "
            + (path != null ? path.name : "null"));
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
    System.out.println(
        "[DriveToPath] Constructor without interrupter called for path: "
            + (path != null ? path.name : "null"));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[DriveToPath] Initializing path: " + (path != null ? path.name : "null"));
    System.out.println("[DriveToPath] Has interrupter? " + (interrupter != null ? "YES" : "NO"));

    Logger.recordOutput("DriveToPoint/StartPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/Status", "Pathfinding to start of prebuilt path");
    Logger.recordOutput("DriveToPoint/HasInterrupter", interrupter != null);

    // Reset debug variables
    wasInterruptedByTrigger = false;
    lastDebugPrintTime = System.currentTimeMillis();

    // Create pathfinding command to the prebuilt path
    pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints).until(interrupter);

    // Schedule the path following command
    pathFollowingCommand.schedule();
    System.out.println("[DriveToPath] Path following command scheduled");
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

      // Throttle debug printing to avoid flooding console
      long currentTime = System.currentTimeMillis();
      if (currentTime - lastDebugPrintTime > DEBUG_PRINT_INTERVAL_MS) {
        System.out.println("[DriveToPath] Execute: Interrupter state = " + currentInterruptState);
        System.out.println(
            "[DriveToPath] PathFollowingCommand isFinished = "
                + (pathFollowingCommand != null ? pathFollowingCommand.isFinished() : "null"));
        lastDebugPrintTime = currentTime;
      }

      // Record if we were interrupted by trigger for later use
      if (currentInterruptState) {
        wasInterruptedByTrigger = true;
        System.out.println("[DriveToPath] INTERRUPT DETECTED! Trigger activated!");
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(
        "[DriveToPath] Command ending. Interrupted: "
            + interrupted
            + ", Trigger activated: "
            + wasInterruptedByTrigger);

    if (pathFollowingCommand != null) {
      pathFollowingCommand.cancel();
      System.out.println("[DriveToPath] Path following command cancelled");
    }

    if (interrupted || wasInterruptedByTrigger) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted");
      System.out.println("[DriveToPath] Path execution was interrupted");
    } else {
      Logger.recordOutput("DriveToPoint/Status", "Successfully followed path");
      System.out.println("[DriveToPath] Path execution completed successfully");
    }

    // Log final status
    Logger.recordOutput("DriveToPoint/FinalPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/WasInterruptedByTrigger", wasInterruptedByTrigger);

    System.out.println("[DriveToPath] Final pose: " + drive.getPose());

    // Make sure we stop the drive when the command ends
    drive.stop();
    System.out.println("[DriveToPath] Drive stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (interrupter != null && interrupter.getAsBoolean()) {
      System.out.println("[DriveToPath] isFinished returning true: Command interrupted by trigger");
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted by trigger");
      return true;
    }

    boolean pathFinished = pathFollowingCommand != null && pathFollowingCommand.isFinished();
    if (pathFinished) {
      System.out.println(
          "[DriveToPath] isFinished returning true: Path following command completed");
    }

    return pathFinished;
  }
}
