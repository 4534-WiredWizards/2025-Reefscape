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

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

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

  /* Game Field Constants */
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

  /* Scoring Enums & Configuration */
  public enum ReefZone {
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4,
    ZONE_5,
    ZONE_6
  }

  public enum ScoringSide {
    LEFT,
    RIGHT
  }

  public enum ScoringHeight {
    L1(Elevator.POSITION_GROUND, Wrist.L1_ANGLE),
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

  interface ScoringPositions {
    record ZonePosition(double x, double y, double theta) {}

    // Zone positions for each side (Left and Right)
    static final Map<ReefZone, Map<ScoringSide, Pose2d>> ZONE_POSITIONS =
        Map.of(
            ReefZone.ZONE_1,
            Map.of(
                ScoringSide.LEFT, new Pose2d(3.083, 4.189, new Rotation2d(0.0)),
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
    int LEFT_MOTOR_ID = 51;
    int RIGHT_MOTOR_ID = 52;

    // Position Limits
    double MAX_SAFE_POS = 71.0;
    double MIN_SAFE_POS = 0.0;
    double ELEVATOR_DANGER_LIMIT = 6.8;

    // Voltage Configuration
    double PEAK_FORWARD_VOLTAGE = 12.0;
    double PEAK_REVERSE_VOLTAGE = -12.0;
    double ZEROING_VOLTAGE = 5.0;

    // PID and Feedforward
    double KP = 5.0;
    double KI = 0.01;
    double KD = 0.1;
    double KS = 0.25;
    double KV = 0.6;
    double KA = 0.05;
    double KG = 1.18;

    // Motion Profile
    double CRUISE_VELOCITY = 65.0;
    double MAX_ACCELERATION = 145.0;
    double JERK = 200.0;

    // Physical Properties
    double PULLEY_DIAMETER = 120.0 / 25.4; // 120mm in inches
    double GEAR_RATIO = 5.0;

    // Conversion Factors
    double ROTATIONS_TO_INCHES = (Math.PI * PULLEY_DIAMETER) / GEAR_RATIO;
    double INCHES_TO_ROTATIONS = 1.0 / ROTATIONS_TO_INCHES;

    // Tolerances
    double POSITION_TOLERANCE = 1.2 * INCHES_TO_ROTATIONS;
    double STALL_VELOCITY_THRESHOLD = 0.05;

    // Preset Positions
    double POSITION_GROUND = -0.1;
    double POSITION_L1 = 0.0;
    double POSITION_L2 = 15.95;
    double POSITION_L3 = 32.0;
    double POSITION_L4 = 69.81;
    double POSITION_HIGH_ALGAE = POSITION_L3;
    double POSITION_LOW_ALGAE = 15.0;
    double POSITION_BARGE = 70.0;

    // Manual Control
    double MANUAL_SPEED = 0.75;
    int DOWN_DIRECTION = -1;
  }

  interface Wrist {
    // Motor Configuration
    int PIVOT_MOTOR_ID = 53;
    double SPEED_SCALAR = 0.8;
    double GEAR_RATIO = 125.0;
    double ZEROING_SPEED = 0.1;

    // Encoder Configuration
    interface Encoder {
      int PORT = 1;
      int FULL_RANGE = 1024;
      int EXPECTED_ZERO = 0;
      double ABSOLUTE_OFFSET = 0.0;
    }

    // PID Configuration
    double PID_POSITION_TOLERANCE = 2.2;
    double KP = 2.75;
    double KI = 0.0;
    double KD = 0.15;

    // Feedforward
    double KS = 0.25;
    double KG = 0.13;
    double KV = 0.5;
    double KA = 0.01;

    // Motion Profile
    double CRUISE_VELOCITY = 90.0;
    double ACCELERATION = 120.0;
    double JERK = 0.0;

    // Safety Limits
    double MAX_SAFE_VAL = 0.1;
    double MIN_SAFE_VAL = -65.0;
    double MIN_SAFE_ANGLE = 20.0;
    double MAX_SAFE_ANGLE = 218.0;
    double TRUE_ZERO = 216.0;
    double MIN_CLEAR_ELEVATOR_ANGLE = 136.0;

    // Stall Detection
    double STALL_VELOCITY_THRESHOLD = 0.01;
    double STALL_CURRENT_THRESHOLD = 30.0;

    // Level Positions
    double L1_ANGLE = 123.0;
    double L2_ANGLE = 118.0;
    double L3_ANGLE = 118.0;
    double L4_ANGLE = 89.0;

    // Operational Positions
    double ALGAE_INTAKE_ANGLE = 20.0;
    int CORAL_INTAKE_ANGLE = 205;
    int BARGER_POSITION = 27;
    int DRIVE_POSITION = 212;
    int CORAL_MAX_ANGLE = 75;

    // Roller Subsystem
    interface Roller {
      int MOTOR_ID = 54;
      int FIRST_SENSOR_ID = 2;
      int SECOND_SENSOR_ID = 3;

      // Speeds
      double CORAL_INTAKE_SPEED = 0.4;
      double CORAL_OUTTAKE_SPEED = -0.28;
      double ALGAE_INTAKE_SPEED = 0.1;
      double ALGAE_OUTTAKE_SPEED = -0.5;
    }
  }

  interface Swerve {
    interface Drive {
      double ROTATION_SPEED_SCALAR = 0.5; // 6.0/12.0

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
    int MOTOR_ID = 55;
    int GEAR_RATIO = 25;
    double MANUAL_SPEED_SCALAR = 0.5;
  }

  interface LEDConstants {
    int LED_ID = 26;
    double BRIGHTNESS_SCALAR = 0.4;
    int TIMEOUT_MS = 100;
    int CANDLE_LED_START_INDEX = 0;
    int CANDLE_LED_SEGMENT_SIZE = 8;
  }

  /* Input/Output Configuration */
  interface IO {
    int DRIVER_CONTROLLER_PORT = 0;
    int OPERATOR_CONTROLLER_PORT = 1;

    // Driver Controller (Joystick)
    interface Driver {
      // Axes
      int DRIVE_X_AXIS = 0;
      int DRIVE_Y_AXIS = 1;
      int DRIVE_ROTATE_AXIS = 3;
      int DRIVE_THROTTLE_AXIS = 2;

      interface RightJoystick {
        int RIGHT_THUMB_BUTTON = 4;
        int STRIPED_CENTER_BUTTON = 2;
        int WEIRD_UNDER_BUTTON = 3;
        int TRIGGER = 1;
      }

      interface LeftThrottle {
        int TOP_THUMB_BUTTON = 5;
        int MIDDLE_THUMB_BUTTON = 6;
        int BOTTOM_THUMB_BUTTON = 7;
        int FRONT_THUMB_BUTTON = 8;
        int BACK_POINTER_HIGH_BUTTON = 9;
        int BACK_POINTER_LOW_BUTTON = 10;
      }

      int BASE_LEFT_BUTTON = 11;
      int BASE_RIGHT_BUTTON = 12;
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
      int ZERO_ELEVATOR_BUTTON = 7; // Start button
      int RESET_BOT_POSE_BUTTON = 8; // Select button

      // Axes
      int RIGHT_THUMB_AXIS = 4;
    }
  }

  /* Autonomous Configuration */
  interface Autonomous {
    // Robot Physical Properties
    double ROBOT_MASS_KG = 61.68;
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
