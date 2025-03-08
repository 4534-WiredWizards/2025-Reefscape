// Copyright 2021-2025 FRC 4534
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
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

/**
 * Contains global constants and configurations for the robot. Includes runtime mode definitions,
 * subsystem configurations, and controller mappings.
 */
public interface Constants {

  /* Runtime Mode Configuration */
  public static enum Mode {
    /** Running on a real robot */
    REAL,
    /** Running in physics simulation */
    SIM,
    /** Replaying from log file */
    REPLAY
  }

  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  // LED constants
  interface LEDConstants {

    int LED_ID = 26;
    double brightnessScalar = 0.4;
    int timeoutMs = 100;
    int CANdleLEDstartIndex = 0;
    int CANdleLEDSegmentSize = 8;
  }

  // Field Positions
  interface FieldPosition {

    interface Blue {

      interface Reef {

        double CENTER_X = 4.5;
        double CENTER_Y = 4.0;
      }
    }

    interface Red {

      interface Reef {

        double CENTER_X = 13.0;
        double CENTER_Y = 4.0;
      }
    }
  }

  // Scoring Enums
  public enum ReefZone {
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4,
    ZONE_5,
    ZONE_6,
  }

  public enum ScoringSide {
    LEFT,
    RIGHT
  }

  public enum ScoringHeight {
    L1(Elevator.POSITION_GROUND, Wrist.L1_ANGLE), // elevator position, wrist angle
    L2(Elevator.POSITION_L2, Wrist.L2_ANGLE),
    L3(Elevator.POSITION_L3, Wrist.L3_ANGLE),
    L4(Elevator.POSITION_L4, Wrist.L4_ANGLE);

    private final double elevatorPosition;
    private final double wristAngle;

    ScoringHeight(double elevatorPosition, double wristAngle) {
      this.elevatorPosition = elevatorPosition;
      this.wristAngle = wristAngle;
    }

    public double getElevatorPosition() {
      return elevatorPosition;
    }

    public double getWristAngle() {
      return wristAngle;
    }
  }

  // Scoring Positions
  interface ScoringPositions {
    // Zone position configuration

    record ZonePosition(double x, double y, double theta) {}

    // Zone positions for each side (Left and Right)
    static final Map<ReefZone, Map<ScoringSide, Pose2d>> ZONE_POSITIONS =
        Map.of(
            ReefZone.ZONE_1,
            Map.of(
                ScoringSide.LEFT, new Pose2d(3.083, 4.189, new Rotation2d(0.0)), // x, y, theta
                ScoringSide.RIGHT, new Pose2d(3.083, 3.859, new Rotation2d(0.0))),
            ReefZone.ZONE_2,
            Map.of(
                ScoringSide.LEFT, new Pose2d(3.647, 2.914, new Rotation2d(Math.toRadians(60.0))),
                ScoringSide.RIGHT, new Pose2d(3.946, 2.754, new Rotation2d(Math.toRadians(60.0)))),
            ReefZone.ZONE_3,
            Map.of(
                ScoringSide.LEFT, new Pose2d(5.034, 2.742, new Rotation2d(Math.toRadians(120.0))),
                ScoringSide.RIGHT, new Pose2d(5.316, 2.923, new Rotation2d(Math.toRadians(120.0)))),
            ReefZone.ZONE_4,
            Map.of(
                ScoringSide.LEFT, new Pose2d(5.894, 3.858, new Rotation2d(Math.toRadians(180.0))),
                ScoringSide.RIGHT, new Pose2d(5.894, 4.189, new Rotation2d(Math.toRadians(180.0)))),
            ReefZone.ZONE_5,
            Map.of(
                ScoringSide.LEFT, new Pose2d(5.314, 5.127, new Rotation2d(Math.toRadians(-120.0))),
                ScoringSide.RIGHT,
                    new Pose2d(5.022, 5.268, new Rotation2d(Math.toRadians(-120.0)))),
            ReefZone.ZONE_6,
            Map.of(
                ScoringSide.LEFT, new Pose2d(3.957, 5.263, new Rotation2d(Math.toRadians(-60.0))),
                ScoringSide.RIGHT,
                    new Pose2d(3.673, 5.108, new Rotation2d(Math.toRadians(-60.0)))));

    static Pose2d getPose(ReefZone zone, ScoringSide side) {
      return ZONE_POSITIONS.get(zone).get(side);
    }
  }

  /* Subsystem Constants */
  interface Elevator {
    // Motor IDs

    public static final int LEFT_MOTOR_ID = 51; // Adjust as needed
    public static final int RIGHT_MOTOR_ID = 52; // Adjust as needed

    // Elevator Position Limits
    public static final double MAX_SAFE_POS = 128.0; // Maximum safe position (adjust as needed)
    public static final double MIN_SAFE_POS = 0; // Minimum safe position (adjust as needed)

    // Voltage Limits
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
    public static final double ZEROING_VOLTAGE = 2.5;

    // PID and Feedforward Constants
    public static final double KP = 2.5; // Proportional gain
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.1; // Derivative gain

    // Feedforward Constants
    public static final double KS = 0.25; // Static friction compensation
    public static final double KV = 0.60; // Velocity feedforward
    public static final double KA = 0.06; // Acceleration feedforward //Was .12
    public static final double KG = 1.18; // Gravity feedforward

    // Motion Magic Constants
    // public static final double CRUISE_VELOCITY = 110.0; // In rotations/sec (~39.58 in/s)
    // public static final double MAX_ACCELERATION = 80.0; // In rotations/sec²
    // public static final double JERK = 600.0; // In rotations/sec³

    // Oh- We are going fast as HECK
    public static final double CRUISE_VELOCITY =
        128.0; // In units/sec full speed for 1s bottom to top
    public static final double MAX_ACCELERATION = 256.0; // In units/sec²
    public static final double JERK = 800.0; // In units/sec³

    // Physical Constantsp
    public static final double PULLEY_DIAMETER = 120.0 / 25.4; // 120mm -> ~4.724 inches
    public static final double GEAR_RATIO = 9; // Verified 15:1 ratio

    // Conversion factors
    // Conversion factors (recalculated based on new measurements)
    public static final double ROTATIONS_TO_INCHES =
        (Math.PI * PULLEY_DIAMETER) / GEAR_RATIO; // ~0.9896 in/rot
    public static final double INCHES_TO_ROTATIONS = 1.0 / ROTATIONS_TO_INCHES; // ~1.0105 rot/in

    // Tolerance and Threshold Values
    public static final double POSITION_TOLERANCE = 1.0 * INCHES_TO_ROTATIONS; // ~1 inch tolerance
    public static final double STALL_VELOCITY_THRESHOLD = 0.05; // Rotations/sec

    // Safety Limits
    public static final double ELEVATOR_DANGER_LIMIT = 12.0;

    // Elevator Stop Positions
    // Define preset positions that the elevator can move to
    public static final double POSITION_GROUND = 0.0;
    public static final double POSITION_L1 = 0;
    public static final double POSITION_L2 = 25;
    public static final double POSITION_L3 = 55;
    public static final double POSITION_L4 = 115;
    public static final double POSITION_BARGE = 127;

    // Manual Control Constants
    public static final double MANUAL_SPEED = 0.4; // Speed for manual control
    public static final int DOWN_DIRECTION = -1; // Direction for manual control
  }

  interface Wrist {
    // Hardware Configuration

    int PIVOT_MOTOR_ID = 53;
    double SPEED_SCALAR = 0.8;
    double GEAR_RATIO = 125.0;
    double ZEROING_SPEED = 0.1;

    // Encoder Configuration
    interface Encoder {

      int PORT = 1;
      int FULL_RANGE = 1024;
      int EXPECTED_ZERO = 0;
      double ABSOLUTE_OFFSET = 0.0; // TODO: Calibrate
    }

    // Control System Constants
    // PID Configuration
    double PID_POSITION_TOLERANCE = 1;
    double KP = 2.75;
    double KI = 0.0;
    double KD = 0.15;

    // Feedforward Constants
    double KS = 0.25; // Static friction compensation
    double KG = 0.13; // Gravity compensation
    double KV = 0.5; // Velocity feedforward
    double KA = 0.01; // Acceleration feedforward

    // Motion Profile Configuration
    double CRUISE_VELOCITY = 30; // In raw encoder vlues/s
    double ACCELERATION = 60; // In raw encoder values/s/s
    double JERK = 0;

    // Safety Limits
    double MAX_SAFE_VAL = 0.1; // Upper limit (in rotations) (uses relative encoder)
    double MIN_SAFE_VAL = -65.0; // Lower limit (in rotations) (uses relative encoder)

    // PID Saftey Angles
    double MIN_SAFE_ANGLE = 20.0;
    double MAX_SAFE_ANGLE = 218.0;

    double TRUE_ZERO = 216; // Offset for zero position
    double MIN_CLEAR_ELEVATOR_ANGLE = 136; // Minimum angle to clear elevator

    // Stall Detection
    double STALL_VELOCITY_THRESHOLD = .01;
    double STALL_CURRENT_THRESHOLD = 30;

    // Preset Positions
    // Level Positions
    double L1_ANGLE = 136;
    double L2_ANGLE = 136;
    double L3_ANGLE = 136;
    double L4_ANGLE = 108;

    // Operational Positions
    int CORAL_INTAKE_ANGLE = 205;
    int BARGER_POSITION = 27;
    int DRIVE_POSITION = 212;

    // Game Piece Handling
    int CORAL_MAX_ANGLE = 75;

    // Roller Subsystem
    interface Roller {

      int MOTOR_ID = 54;
      int FIRST_SENSOR_ID = 2;
      int SECOND_SENSOR_ID = 3;

      // Intake/Outtake Speeds
      double CORAL_INTAKE_SPEED = 0.4;
      double CORAL_OUTTAKE_SPEED = -0.2;
      double ALGAE_INTAKE_SPEED = 0.1;
      double ALGAE_OUTTAKE_SPEED = -0.3;
    }
  }

  interface Swerve {

    interface Drive {
      double ROTATION_SPEED_SCALAR = (6.0 / 12.0);

      interface PID {

        double KP = 100.0;
        double KI = 0.0;
        double KD = 0.5;
        double KS = 0.1;
        double KV = 1.59;
        double KA = 0.0;
      }
    }

    interface Steer {

      interface PID {

        double KP = 0.0097625642;
        double KI = 0.0;
        double KD = 0.0;
        double KS = 0.19258;
        double KV = 0.0194598749;
        double KA = 0.0027024509;
      }
    }
  }

  interface Climb {
    // Motor ID
    int MOTOR_ID = 55;

    int GEAR_RATIO = 25;

    // Manual Speed Scalar
    double MANUAL_SPEED_SCALAR = 0.5;
  }

  /* Input/Output Configuration */
  interface IO {
    // Controller Port Configuration

    static final int DRIVER_CONTROLLER_PORT = 0;
    static final int OPERATOR_CONTROLLER_PORT = 1;

    // Driver Controller (Xbox)
    interface Driver {
      // Axes

      int DRIVE_X_AXIS = 0;
      int DRIVE_Y_AXIS = 1;
      int DRIVE_ROTATE_AXIS = 3;
      int DRIVE_THROTTLE_AXIS = 2;

      // Buttons
      int ZERO_GYRO_BUTTON = 2; // A button
      int FIELD_RELATIVE_TOGGLE = 3; // B button
      int SLOW_MODE_TOGGLE = 4; // X button
      int LOCK_ANGLE_BUTTON = 7;
      int STOP_WITH_X_BUTTON = 8;
      int TRIGGER = 1;
    }

    // Operator Controller (Xbox)
    interface Operator {
      // Buttons

      int ELEVATOR_HOME_BUTTON = 1; // A button
      int INTAKE_CORAL_BUTTON = 2; // B button
      int SCORE_L1_BUTTON = 3; // X button
      int SCORE_L2_BUTTON = 4; // Y button
      int MANUAL_OVERRIDE_BUTTON = 5; // Left bumper
      int CLIMB_SEQUENCE_BUTTON = 6; // Right bumper
      int RIGHT_THUMB_AXIS = 4;
    }
  }

  /* Autonomous Configuration */
  interface Autonomous {
    // Pathfinding Constants

    double ROBOT_MASS_KG = 55.79;
    double ROBOT_MOI = 6.883;
    double WHEEL_COF = 1.2;

    // PID Constants
    interface PID {

      double DRIVE_KP = 5.0;
      double DRIVE_KI = 0.0;
      double DRIVE_KD = 0.0;

      double TURN_KP = 2.0;
      double TURN_KI = 1.0;
      double TURN_KD = 0.0;
    }
  }
}
