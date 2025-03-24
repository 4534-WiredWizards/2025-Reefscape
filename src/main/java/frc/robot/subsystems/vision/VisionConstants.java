package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.ReefZone;
import java.util.Map;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-front";
  public static String camera1Name = "limelight-back";

  // Map of zone to primary AprilTag ID for Blue alliance
  public static final Map<ReefZone, Integer> BLUE_ZONE_PRIMARY_TAGS =
      Map.of(
          ReefZone.ZONE_1, 18,
          ReefZone.ZONE_2, 17,
          ReefZone.ZONE_3, 22,
          ReefZone.ZONE_4, 21,
          ReefZone.ZONE_5, 20,
          ReefZone.ZONE_6, 19);

  // Map of zone to primary AprilTag ID for Red alliance
  public static final Map<ReefZone, Integer> RED_ZONE_PRIMARY_TAGS =
      Map.of(
          ReefZone.ZONE_1, 7,
          ReefZone.ZONE_2, 8,
          ReefZone.ZONE_3, 9,
          ReefZone.ZONE_4, 10,
          ReefZone.ZONE_5, 11,
          ReefZone.ZONE_6, 6);

  // Basic filtering thresholds
  public static double maxAmbiguity =
      0.2; // Lower = more strict (reject more ambiguous measurements)
  public static double maxZError = 0.75; // Reduced from 8 to be more strict

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline =
      0.02; // Meters (balance between 0.01 from old and 0.03 from new)
  public static double angularStdDevBaseline =
      0.06; // Radians (balance between 0.06 from old and 0.08 from new)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // (lower standard deviation = higher trust)
  public static double[] cameraStdDevFactors =
      new double[] {
        .8, // Camera 0 - front
        1.2 // Camera 1 - back (slightly less trusted than front)
      };

  // Multipliers to apply for MegaTag 2 observations
  // Using high but finite values to allow some influence without overpowering odometry
  public static double linearStdDevMegatag2Factor =
      3.0; // More stable than full 3D solve but not perfect
  public static double angularStdDevMegatag2Factor = 3.0; // Should use robot gyro for rotation

  // Maximum pose difference in meters to accept a vision measurement
  public static final double maxPoseDifference = 1.5;
  // Add this line under the maxTagDistance declaration
  public static final double minTagDistance =
      0.01; // Minimum meter distance to accept vision measurements

  // Maximum meter distance to accept vision measurements
  public static final double maxTagDistance = 5;

  // Maximum gyro rate in rad/s to accept vision measurements
  public static final double maxGyroRate = Math.toRadians(720);

  // Maximum time difference in seconds to consider a vision measurement fresh
  public static final double maxTimeDifference = 0.8;
}
