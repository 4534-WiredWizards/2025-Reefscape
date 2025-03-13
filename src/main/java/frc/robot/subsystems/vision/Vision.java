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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
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

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
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
                || observation.averageTagDistance() > 4.0;

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
    if (consumer instanceof PoseProvider) {
      return ((PoseProvider) consumer).getCurrentPose();
    }
    // Default to origin if consumer can't provide pose
    return new Pose2d();
  }

  /**
   * Gets the current gyro rate from the consumer (assuming consumer implements a method to return
   * it) This may need to be modified based on your actual consumer implementation
   */
  private double getGyroRateFromConsumer() {
    if (consumer instanceof GyroRateProvider) {
      return ((GyroRateProvider) consumer).getGyroRate();
    }
    // Default to 0 if consumer can't provide gyro rate
    return 0.0;
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
    if (observation.averageTagDistance() > 4.0) {
      return "Tags too far: " + observation.averageTagDistance() + "m";
    }

    return "Unknown reason";
  }

  /** Optional interface for consumers that can provide current pose */
  public interface PoseProvider {
    Pose2d getCurrentPose();
  }

  /** Optional interface for consumers that can provide gyro rate */
  public interface GyroRateProvider {
    double getGyroRate();
  }

  /** Functional interface for vision consumers */
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Resets robot pose using vision if a valid measurement is available. Similar to the
   * resetLimelightBotPoseBlue method from the old implementation.
   */
  public void resetRobotPose() {
    Logger.recordOutput("Vision/Status", "Attempting to reset pose from vision...");

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      if (!inputs[cameraIndex].connected) {
        Logger.recordOutput("Vision/Status", "Camera " + cameraIndex + " not connected");
        continue;
      }

      // Find best observation (most tags, closest)
      VisionIO.PoseObservation bestObservation = null;
      int maxTagCount = 0;
      double minDistance = Double.MAX_VALUE;

      for (var observation : inputs[cameraIndex].poseObservations) {
        // Skip invalid observations
        if (observation.tagCount() == 0
            || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
            || Math.abs(observation.pose().getZ()) > maxZError
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth()
            || observation.averageTagDistance() > 4.0) {
          continue;
        }

        // Check if data is fresh
        double currentTime = Timer.getFPGATimestamp();
        if (Math.abs(currentTime - observation.timestamp()) > maxTimeDifference) {
          Logger.recordOutput(
              "Vision/Status",
              "Stale data for reset on Camera "
                  + cameraIndex
                  + ": Delta = "
                  + (currentTime - observation.timestamp()));
          continue;
        }

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
        if (consumer instanceof PoseProvider) {
          // Assuming consumer can also set the pose
          if (consumer instanceof PoseSetter) {
            ((PoseSetter) consumer).setPose(bestObservation.pose().toPose2d());
            Logger.recordOutput(
                "Vision/Status",
                "Successfully reset pose to "
                    + bestObservation.pose().toPose2d()
                    + " from Camera "
                    + cameraIndex
                    + " with "
                    + bestObservation.tagCount()
                    + " tags");
            return;
          }
        }
      }
    }

    Logger.recordOutput("Vision/Status", "Could not reset pose: no valid vision data found.");
  }

  /** Optional interface for consumers that can set the current pose */
  public interface PoseSetter {
    void setPose(Pose2d pose);
  }
}
