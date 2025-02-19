// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  private final Drive swerveDrive;
  private final String[] limelights = {"limelight-front"};
  private final int[] validAprilTagIDs = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
  };

  public VisionSubsystem(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
    configureLimelights();
  }

  private void configureLimelights() {
    for (String ll : limelights) {
      try {
        LimelightHelpers.SetFiducialIDFiltersOverride(ll, validAprilTagIDs);
        LimelightHelpers.SetIMUMode(ll, 0);
      } catch (Exception e) {
        System.err.println("Error configuring Limelight " + ll + ": " + e.getMessage());
        Logger.recordOutput("Limelight/" + ll + "/Connected", false);
      }
    }
  }

  @Override
  public void periodic() {
    for (String ll : limelights) {
      try {
        updateVisionPose(ll);
        Logger.recordOutput("Limelight/" + ll + "/Connected", true);
      } catch (Exception e) {
        System.err.println("Error updating pose from Limelight " + ll + ": " + e.getMessage());
        Logger.recordOutput("Limelight/" + ll + "/Connected", false);
      }
    }
  }

  private void updateVisionPose(String limelightName) {
    Pose2d currentEstimate = swerveDrive.getPose();
    try {
      LimelightHelpers.SetRobotOrientation(
          limelightName, currentEstimate.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    } catch (Exception e) {
      System.err.println("Error setting orientation for " + limelightName + ": " + e.getMessage());
      return;
    }

    PoseEstimate mt2;
    try {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    } catch (Exception e) {
      System.err.println("Error getting pose from " + limelightName + ": " + e.getMessage());
      return;
    }

    if (mt2 == null) {
      return;
    }

    Logger.recordOutput("Vision/Timestamp", mt2.timestampSeconds);

    if (shouldAcceptMeasurement(mt2)) {
      Matrix<N3, N1> stdDevs = calculateMeasurementUncertainty(mt2);
      swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }
  }

  private boolean shouldAcceptMeasurement(PoseEstimate estimate) {
    if (estimate.tagCount == 0) {
      return false;
    }
    if (estimate.avgTagDist > 4.0) {
      return false;
    }
    if (Math.abs(swerveDrive.getGyroRate()) > Math.toRadians(720)) {
      return false;
    }

    Pose2d currentEstimate = swerveDrive.getPose();
    double poseDifference =
        currentEstimate.getTranslation().getDistance(estimate.pose.getTranslation());
    return poseDifference <= 1.5;
  }

  private Matrix<N3, N1> calculateMeasurementUncertainty(PoseEstimate estimate) {
    double xyStdDev = 0.01 * estimate.avgTagDist * (2.0 / estimate.tagCount);
    double rotStdDev = Math.toRadians(0.5 * estimate.avgTagDist * (2.0 / estimate.tagCount));
    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  public void resetLimelightBotPoseBlue() {
    Logger.recordOutput("Vision/Status", "Attempting to reset bot pose from Limelight...");
    for (String ll : limelights) {
      try {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

        if (estimate == null) {
          Logger.recordOutput("Vision/Status", "No pose estimate available from " + ll);
          continue;
        }

        if (estimate.tagCount == 0) {
          Logger.recordOutput("Vision/Status", "No tags detected for reset on " + ll);
          continue;
        }
        if (estimate.avgTagDist > 4.0) {
          Logger.recordOutput(
              "Vision/Status",
              "Avg tag distance too far for reset on " + ll + ": " + estimate.avgTagDist);
          continue;
        }
        double currentTime = Timer.getFPGATimestamp();
        if (Math.abs(currentTime - estimate.timestampSeconds) > 0.3) {
          Logger.recordOutput(
              "Vision/Status",
              "Stale data for reset on "
                  + ll
                  + ": Delta = "
                  + (currentTime - estimate.timestampSeconds));
          continue;
        }
        swerveDrive.setPose(estimate.pose);
        Logger.recordOutput("Vision/Status", "Successfully reset pose to " + estimate.pose + " from " + ll);
        return;
      } catch (Exception e) {
        Logger.recordOutput("Vision/Status", "Error during reset from " + ll + ": " + e.getMessage());
      }
    }
    Logger.recordOutput("Vision/Status", "Could not reset pose: no valid Limelight data found.");
  }
}