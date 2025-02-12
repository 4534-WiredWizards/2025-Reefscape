// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  private final Drive swerveDrive;
  private final String[] limelights = {"limelight-front" /*  "limelight-back"*/};
  private final int[] validAprilTagIDs = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
  };

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
    configureLimelights();
  }

  private void configureLimelights() {
    for (String ll : limelights) {
      // Set valid AprilTag IDs and IMU mode (0 = use external gyro)
      LimelightHelpers.SetFiducialIDFiltersOverride(ll, validAprilTagIDs);
      LimelightHelpers.SetIMUMode(ll, 0); // Use external gyro data
    }
  }

  @Override
  public void periodic() {
    System.out.println("in periodic");
    for (String ll : limelights) {
      updateVisionPose(ll);
      System.out.println("in forloop " + ll);
    }
  }

  private void updateVisionPose(String limelightName) {
    // Get current robot pose estimate
    Pose2d currentEstimate = swerveDrive.getPose();

    // Set robot orientation for MegaTag2 (critical for single-tag operation)
    LimelightHelpers.SetRobotOrientation(
        limelightName, currentEstimate.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // Get MegaTag2 pose estimate (always use blue origin for 2024+)
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    SmartDashboard.putNumber("mt2 X", mt2.pose.getX());
    SmartDashboard.putNumber("mt2 Y", mt2.pose.getY());

    // Validate and apply vision measurement
    if (true /*shouldAcceptMeasurement(mt2) */) {
      Matrix<N3, N1> stdDevs = calculateMeasurementUncertainty(mt2);
      swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
      System.out.println("Vision measurement accepted!");
    }
  }

  private boolean shouldAcceptMeasurement(PoseEstimate estimate) {
    // Basic validation checks
    if (estimate.tagCount == 0) return false; // Reject if no tags are detected
    if (estimate.avgTagDist > 4.0)
      return false; // Reject if the average distance to tags is greater than 4 meters

    // Motion sanity checks
    if (Math.abs(swerveDrive.getGyroRate()) > Math.toRadians(720))
      return false; // Reject if the yaw velocity is greater than 720 degrees per second
    // AKA we are spinnger super fast and probably should not trust the vision data

    // Pose consistency check (optional)
    Pose2d currentEstimate = swerveDrive.getPose();
    double poseDifference =
        currentEstimate.getTranslation().getDistance(estimate.pose.getTranslation());
    return poseDifference < 0.5; // Reject if the pose difference is greater than 0.5 meters
  }

  private Matrix<N3, N1> calculateMeasurementUncertainty(PoseEstimate estimate) {
    // Dynamic uncertainty based on tag observations
    double xyStdDev = 0.01 * estimate.avgTagDist * (2.0 / estimate.tagCount);
    double rotStdDev = Math.toRadians(0.5 * estimate.avgTagDist * (2.0 / estimate.tagCount));
    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }
}
