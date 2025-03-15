package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReefZone;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Drive driveSystem;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(Drive driveSystem, VisionIO... io) {
    this.driveSystem = driveSystem;
    this.consumer = driveSystem::addVisionMeasurement; // Use the method reference here
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    // Get current zone for tag filtering
    ReefZone currentZone = driveSystem.getZone();
    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    // Get the tag ID for the current zone
    Integer currentZoneTagId =
        isRedAlliance
            ? RED_ZONE_PRIMARY_TAGS.get(currentZone)
            : BLUE_ZONE_PRIMARY_TAGS.get(currentZone);

    // Log the current zone and tag
    Logger.recordOutput("Vision/CurrentZone", currentZone.toString());
    Logger.recordOutput(
        "Vision/CurrentZoneTagId", currentZoneTagId != null ? currentZoneTagId : -1);

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Get current pose from consumer (assumes consumer can provide current pose)
    Pose2d currentPose = getCurrentPoseFromConsumer();
    double currentGyroRate = getGyroRateFromConsumer();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses - ONLY add tags that match the current zone's tag ID
      for (int tagId : inputs[cameraIndex].tagIds) {
        // Skip tags that don't match the current zone's tag
        if (currentZoneTagId != null && tagId != currentZoneTagId) {
          Logger.recordOutput("Vision/SkippedTag", tagId);
          continue;
        }

        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Get current time for freshness check
        double currentTime = Timer.getFPGATimestamp();

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()

                // Check pose difference with current estimate
                || currentPose
                        .getTranslation()
                        .getDistance(observation.pose().toPose2d().getTranslation())
                    > maxPoseDifference

                // Check if robot is rotating too fast
                || Math.abs(currentGyroRate) > maxGyroRate

                // Check if data is fresh
                || Math.abs(currentTime - observation.timestamp()) > maxTimeDifference

                // Check tag distance (reject if too far)
                || observation.averageTagDistance() > maxTagDistance;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          Logger.recordOutput(
              "Vision/Camera" + Integer.toString(cameraIndex) + "/RejectionReason",
              getRejectionReason(observation, currentPose, currentGyroRate, currentTime));
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        // Adjust standard deviations based on observation type
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Apply camera-specific factors
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Apply additional trust factors based on tag count
        // Increase trust (lower std dev) with more tags
        if (observation.tagCount() > 1) {
          linearStdDev /= Math.sqrt(observation.tagCount());
          angularStdDev /= Math.sqrt(observation.tagCount());
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /**
   * Gets the current pose from the consumer (assuming consumer implements a method to return it)
   * This may need to be modified based on your actual consumer implementation
   */
  private Pose2d getCurrentPoseFromConsumer() {
    return driveSystem.getPose();
  }

  /**
   * Gets the current gyro rate from the consumer (assuming consumer implements a method to return
   * it) This may need to be modified based on your actual consumer implementation
   */
  private double getGyroRateFromConsumer() {
    return driveSystem.getGyroRate();
  }

  /** Generates a detailed rejection reason for logging */
  private String getRejectionReason(
      VisionIO.PoseObservation observation,
      Pose2d currentPose,
      double currentGyroRate,
      double currentTime) {

    if (observation.tagCount() == 0) {
      return "No tags detected";
    }
    if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
      return "Single tag with high ambiguity: " + observation.ambiguity();
    }
    if (Math.abs(observation.pose().getZ()) > maxZError) {
      return "Z error too high: " + observation.pose().getZ();
    }
    if (observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
      return "Pose outside field: " + observation.pose().toPose2d();
    }
    double poseDifference =
        currentPose.getTranslation().getDistance(observation.pose().toPose2d().getTranslation());
    if (poseDifference > maxPoseDifference) {
      return "Pose difference too large: " + poseDifference;
    }
    if (Math.abs(currentGyroRate) > maxGyroRate) {
      return "Gyro rate too high: " + Math.toDegrees(currentGyroRate) + " deg/s";
    }
    double timeDifference = Math.abs(currentTime - observation.timestamp());
    if (timeDifference > maxTimeDifference) {
      return "Stale data: " + timeDifference + "s old";
    }
    if (observation.averageTagDistance() > maxTagDistance) {
      return "Tags too far: " + observation.averageTagDistance() + "m";
    }

    return "Unknown reason";
  }

  /**
   * Resets robot pose using vision if a valid measurement is available. Similar to the
   * resetLimelightBotPoseBlue method from the old implementation.
   */
  public void resetRobotPose() {
    Logger.recordOutput("Vision/Status", "Attempting to reset pose from vision...");
    boolean foundAnyObservations = false;

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      if (!inputs[cameraIndex].connected) {
        Logger.recordOutput("Vision/Status", "Camera " + cameraIndex + " not connected");
        continue;
      }

      Logger.recordOutput(
          "Vision/Status",
          "Checking camera "
              + cameraIndex
              + " with "
              + inputs[cameraIndex].poseObservations.length
              + " observations");

      // Find best observation (most tags, closest)
      VisionIO.PoseObservation bestObservation = null;
      int maxTagCount = 0;
      double minDistance = Double.MAX_VALUE;

      for (var observation : inputs[cameraIndex].poseObservations) {
        foundAnyObservations = true;
        // Log each observation for debugging
        Logger.recordOutput(
            "Vision/ResetAttempt/Camera" + cameraIndex + "/Observation",
            "Tags: "
                + observation.tagCount()
                + ", Ambiguity: "
                + observation.ambiguity()
                + ", Z: "
                + observation.pose().getZ()
                + ", Dist: "
                + observation.averageTagDistance()
                + ", Pose: "
                + observation.pose().toPose2d());

        // Relaxed validation criteria - only check critical issues
        boolean isValid = true;
        String rejectionReason = "";

        // Still need at least one tag
        if (observation.tagCount() == 0) {
          isValid = false;
          rejectionReason = "No tags detected";
        }
        // Relaxed single tag ambiguity threshold (1.5x previous value)
        else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity * 1.5) {
          isValid = false;
          rejectionReason = "Single tag with extreme ambiguity: " + observation.ambiguity();
        }
        // Still check if we're on the field, but with a small buffer outside
        else if (observation.pose().getX() < -0.5
            || observation.pose().getX() > aprilTagLayout.getFieldLength() + 0.5
            || observation.pose().getY() < -0.5
            || observation.pose().getY() > aprilTagLayout.getFieldWidth() + 0.5) {
          isValid = false;
          rejectionReason = "Pose far outside field: " + observation.pose().toPose2d();
        }
        // Relaxed Z error check (1.5x previous threshold)
        else if (Math.abs(observation.pose().getZ()) > maxZError * 1.5) {
          isValid = false;
          rejectionReason = "Z error extremely high: " + observation.pose().getZ();
        }
        // Relaxed distance check (1.25x previous threshold)
        else if (observation.averageTagDistance() > maxTagDistance * 1.25) {
          isValid = false;
          rejectionReason = "Tags very far: " + observation.averageTagDistance() + "m";
        }

        // Check if data is fresh - relaxed threshold (2x previous value)
        double currentTime = Timer.getFPGATimestamp();
        double timeDifference = Math.abs(currentTime - observation.timestamp());
        if (timeDifference > maxTimeDifference * 2) {
          isValid = false;
          rejectionReason = "Very stale data: " + timeDifference + "s old";
        }

        if (!isValid) {
          Logger.recordOutput(
              "Vision/ResetAttempt/Camera" + cameraIndex + "/Rejected", rejectionReason);
          continue;
        }

        Logger.recordOutput(
            "Vision/ResetAttempt/Camera" + cameraIndex + "/Valid",
            "Tags: " + observation.tagCount() + ", Distance: " + observation.averageTagDistance());

        // Prioritize by tag count, then by distance
        if (observation.tagCount() > maxTagCount
            || (observation.tagCount() == maxTagCount
                && observation.averageTagDistance() < minDistance)) {
          bestObservation = observation;
          maxTagCount = observation.tagCount();
          minDistance = observation.averageTagDistance();
        }
      }

      // Apply best observation if found
      if (bestObservation != null) {
        driveSystem.setPose(bestObservation.pose().toPose2d());
        Logger.recordOutput(
            "Vision/Status",
            "Successfully reset pose to "
                + bestObservation.pose().toPose2d()
                + " from Camera "
                + cameraIndex
                + " with "
                + bestObservation.tagCount()
                + " tags at "
                + bestObservation.averageTagDistance()
                + "m distance");
        return;
      }
    }

    if (!foundAnyObservations) {
      Logger.recordOutput(
          "Vision/Status", "Could not reset pose: no vision observations available.");
    } else {
      Logger.recordOutput(
          "Vision/Status",
          "Could not reset pose: no valid vision data found. Check Vision/ResetAttempt logs for details.");
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
