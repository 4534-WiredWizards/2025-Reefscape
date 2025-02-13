// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  private final Drive swerveDrive;
  private final String[] limelights = {"limelight-front", "limelight-back"};
  private final int[] validAprilTagIDs = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
  };

  public VisionSubsystem(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
    configureLimelights();
  }

  private void configureLimelights() {
    for (String ll : limelights) {
      LimelightHelpers.SetFiducialIDFiltersOverride(ll, validAprilTagIDs);
      LimelightHelpers.SetIMUMode(ll, 0);
    }
  }

  @Override
  public void periodic() {
    for (String ll : limelights) {
      updateVisionPose(ll);
    }

    Pose2d estimatedPose = swerveDrive.getPose();
    // System.out.println("[VisionSubsystem] Current Estimated Pose: " + estimatedPose);
  }

  private void updateVisionPose(String limelightName) {
    Pose2d currentEstimate = swerveDrive.getPose();
    LimelightHelpers.SetRobotOrientation(
        limelightName, currentEstimate.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    double currentTime = Timer.getFPGATimestamp();
    if (Math.abs(currentTime - mt2.timestampSeconds) > 0.3) {
      // System.out.println(
      //     "WARNING: Stale vision data. Delta: " + (currentTime - mt2.timestampSeconds));
    }

    // System.out.println("Vision Pose: " + mt2.pose + " | Timestamp: " + mt2.timestampSeconds);
    SmartDashboard.putNumber("VisionTS", mt2.timestampSeconds);

    if (shouldAcceptMeasurement(mt2)) {
      Matrix<N3, N1> stdDevs = calculateMeasurementUncertainty(mt2);
      // System.out.println("Applying vision. Std Devs: " + stdDevs);
      swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    } else {
      // System.out.println(
      //     "REJECTED - Tag Count: "
      //         + mt2.tagCount
      //         + " | Avg Dist: "
      //         + mt2.avgTagDist
      //         + " | Gyro Rate: "
      //         + Math.toDegrees(swerveDrive.getGyroRate()));
    }
  }

  private boolean shouldAcceptMeasurement(PoseEstimate estimate) {
    if (estimate.tagCount == 0) {
      // System.out.println("Rejected: No tags detected");
      return false;
    }
    if (estimate.avgTagDist > 4.0) {
      // System.out.println("Rejected: Avg tag distance too far: " + estimate.avgTagDist);
      return false;
    }
    if (Math.abs(swerveDrive.getGyroRate()) > Math.toRadians(720)) {
      // System.out.println("Rejected: Excessive gyro rate");
      return false;
    }

    Pose2d currentEstimate = swerveDrive.getPose();
    double poseDifference =
        currentEstimate.getTranslation().getDistance(estimate.pose.getTranslation());
    if (poseDifference > 1.5) {
      // System.out.println("Rejected: Pose diff " + poseDifference + " meters");
      // System.out.println("Current Pose (2): " + currentEstimate);
      // System.out.println("New Pose (2): " + estimate.pose);
      return false;
    }
    return true;
  }

  private Matrix<N3, N1> calculateMeasurementUncertainty(PoseEstimate estimate) {
    
    return VecBuilder.fill(0.1, 0.1, Math.toRadians(1));
  }

  public void resetLimelightBotPoseBlue() {
    // System.out.println("Resetting bot pose from Limelight");
    for (String ll : limelights) {
      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
      if (estimate.tagCount == 0) {
        // System.out.println("No tags detected for reset on " + ll);
        continue;
      }
      if (estimate.avgTagDist > 4.0) {
        // System.out.println("Avg tag distance too far for reset on " + ll);
        continue;
      }
      double currentTime = Timer.getFPGATimestamp();
      if (Math.abs(currentTime - estimate.timestampSeconds) > 0.3) {
        // System.out.println("Stale data for reset on " + ll);
        continue;
      }
      swerveDrive.setPose(estimate.pose);
      System.out.println("Reset pose to " + estimate.pose + " from " + ll);
      return;
    }
    System.out.println("Could not reset pose: no valid Limelight data");
  }
}
