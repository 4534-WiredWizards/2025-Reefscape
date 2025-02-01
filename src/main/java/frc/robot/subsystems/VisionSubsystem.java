// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final Drive swerveDrive;
    private final String[] limelights = {"limelight-front", "limelight-back"};
    private final int[] validAprilTagIDs = {3, 4}; // Example valid IDs

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
        for (String ll : limelights) {
            updateVisionPose(ll);
        }
    }

    private void updateVisionPose(String limelightName) {
        // Get current robot pose estimate
        Pose2d currentEstimate = swerveDrive.getPoseEstimator().getEstimatedPosition();
        
        // Set robot orientation for MegaTag2 (critical for single-tag operation)
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            currentEstimate.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        // Get MegaTag2 pose estimate (always use blue origin for 2024+)
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Validate and apply vision measurement
        if (shouldAcceptMeasurement(mt2)) {
            Matrix<N3, N1> stdDevs = calculateMeasurementUncertainty(mt2);
            swerveDrive.getPoseEstimator().addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
        }
    }

    private boolean shouldAcceptMeasurement(PoseEstimate estimate) {
        // Basic validation checks
        if (estimate.tagCount == 0) return false;
        if (estimate.avgTagDistance > 4.0) return false; // Reject distant tags
        
        // Motion sanity checks
        if (Math.abs(swerveDrive.getGyro().getRate()) > 720) return false;
        
        // Pose consistency check (optional)
        Pose2d currentEstimate = swerveDrive.getPoseEstimator().getEstimatedPosition();
        double poseDifference = currentEstimate.getTranslation().getDistance(estimate.pose.getTranslation());
        return poseDifference < 0.5; // Reject jumps > 0.5 meters
    }

    private Matrix<N3, N1> calculateMeasurementUncertainty(PoseEstimate estimate) {
        // Dynamic uncertainty based on tag observations
        double xyStdDev = 0.01 * estimate.avgTagDistance * (2.0 / estimate.tagCount);
        double rotStdDev = Math.toRadians(0.5 * estimate.avgTagDistance * (2.0 / estimate.tagCount));
        return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }
}