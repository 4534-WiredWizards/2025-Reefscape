// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // public static String camera0Name = "limelight-front";
  public static String camera1Name = "limelight-back";

  
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3; // Increased from 0.2
  public static double maxZError = 0.5; // Reduced from 8 meters

  // Standard deviation baselines
  public static double linearStdDevBaseline = 0.04; // Increased from 0.03
  public static double angularStdDevBaseline = 0.1; // Increased from 0.08

  // Camera trust factors (lower = more trust)
  public static double[] cameraStdDevFactors = new double[] {0.8, 1.0};

  // MegaTag2 factors
  public static double linearStdDevMegatag2Factor = 0.7; // Was POSITIVE_INFINITY
  public static double angularStdDevMegatag2Factor = 0.9; // Was POSITIVE_INFINITY
}
