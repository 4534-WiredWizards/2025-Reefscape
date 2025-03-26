// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToPoint extends Command {

  private final Drive drive;
  private final Pose2d targetPose;
  private Command pathFindingCommand;
  private final BooleanSupplier interrupter;

  /** Creates a new DriveToPoint with interruption capability. */
  public DriveToPoint(Drive drive, Pose2d targetPose, BooleanSupplier interrupter) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.interrupter = interrupter;

    addRequirements(drive);
  }
  
  /** Creates a new DriveToPoint without interruption capability. */
  public DriveToPoint(Drive drive, Pose2d targetPose) {
    this(drive, targetPose, () -> false);  // Default interrupter that never interrupts
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Record initial state
    Logger.recordOutput("DriveToPoint/StartPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/TargetPose", targetPose);
    Logger.recordOutput("DriveToPoint/Status", "Initializing path to target pose");

    PathConstraints constraints =
    new PathConstraints(
      drive.getMaxLinearSpeedMetersPerSec(), // 100% of max velocity
      drive.getMaxLinearSpeedMetersPerSec() * 2, // 200% of max acceleration
      drive.getMaxAngularSpeedRadPerSec() * 0.7, // 70% of max angular velocity
      drive.getMaxAngularSpeedRadPerSec() * 0.7 // 70% of max angular acceleration
  );
            
    // Create pathfinding command with interrupter
    pathFindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0)
        .until(interrupter);

    pathFindingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update current status
    Logger.recordOutput("DriveToPoint/CurrentPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/InterrupterActive", interrupter.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Cancel the pathfinding command to ensure the robot stops
    if (pathFindingCommand != null) {
      pathFindingCommand.cancel();
    }
    
    // Log end status
    Logger.recordOutput("DriveToPoint/FinalPose", drive.getPose());
    Logger.recordOutput("DriveToPoint/Status", interrupted ? "Command interrupted" : "Path completed");
    
    // Ensure the drive stops
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End if the interrupter is triggered
    if (interrupter.getAsBoolean()) {
      Logger.recordOutput("DriveToPoint/Status", "Command interrupted by trigger");
      return true;
    }
    
    // Otherwise, end when the pathfinding command finishes
    return pathFindingCommand != null && pathFindingCommand.isFinished();
  }
}