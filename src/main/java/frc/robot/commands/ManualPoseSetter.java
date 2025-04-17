package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Command to manually set the robot's pose from dashboard values. This allows for quick
 * recalibration during testing and matches.
 */
public class ManualPoseSetter extends Command {
  private final Drive driveSubsystem;

  public ManualPoseSetter(Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Does not require the drive subsystem, as it only sets the pose once
  }

  @Override
  public void initialize() {
    // Get pose values from SmartDashboard
    double x = SmartDashboard.getNumber("ManualPose/X", 7.2);
    double y = SmartDashboard.getNumber("ManualPose/Y", 4);
    double rotationDegrees = SmartDashboard.getNumber("ManualPose/Rotation", 180.0);

    // Create the pose
    Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));

    // Set the pose
    driveSubsystem.setPose(newPose);

    // Log the new pose
    Logger.recordOutput("ManualPose/SetPose", newPose);
    Logger.recordOutput("ManualPose/SetTime", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
  }

  // Work while disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    // Command finishes immediately after setting the pose
    return true;
  }
}
