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



import edu.wpi.first.wpilibj.RobotBase;
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
    L1(Elevator.L1_POS, Wrist.L1_ANGLE), // elevator position, wrist angle
    L2(Elevator.L2_POS, Wrist.L2_ANGLE),
    L3(Elevator.L3_POS, Wrist.L3_ANGLE),
    L4(Elevator.L4_POS, Wrist.L4_ANGLE);

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
    static final Map<ReefZone, Map<ScoringSide, ZonePosition>> ZONE_POSITIONS =
        Map.of(
            ReefZone.ZONE_1,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(3.083, 4.189, 0.0), // x, y, theta
                    ScoringSide.RIGHT, new ZonePosition(3.083, 3.859, 0.0)),
            ReefZone.ZONE_2,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(3.647, 2.914, 60.0),
                    ScoringSide.RIGHT, new ZonePosition(3.946, 2.754, 60.0)),
            ReefZone.ZONE_3,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(5.034, 2.742, 120.0),
                    ScoringSide.RIGHT, new ZonePosition(5.316, 2.923, 120.0)),
            ReefZone.ZONE_4,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(5.894, 3.858, 180.0),
                    ScoringSide.RIGHT, new ZonePosition(5.894, 4.189, 180.0)),
            ReefZone.ZONE_5,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(5.314, 5.127, -120.0),
                    ScoringSide.RIGHT, new ZonePosition(5.022, 5.268, -120.0)),
            ReefZone.ZONE_6,
                Map.of(
                    ScoringSide.LEFT, new ZonePosition(3.957, 5.263, -60.0),
                    ScoringSide.RIGHT, new ZonePosition(3.673, 5.108, -60.0)));

    /** Returns the position for a given zone and side. */
    static ZonePosition getZonePosition(ReefZone zone, ScoringSide side) {
      return ZONE_POSITIONS.get(zone).get(side);
    }
  }

  /* Subsystem Constants */
  interface Elevator {
    // Motor Configurations
    int LEFT_MOTOR_ID = 51;
    int RIGHT_MOTOR_ID = 52;
    int BOTTOM_LIMIT_SWITCH_ID = 0;

    // PID Configuration
    double PID_POSITION_TOLERANCE = 0.1;
    double PID_VELOCITY_TOLERANCE = 1;
    double KP = 0.1;
    double KI = 0.0;
    double KD = 0.0;

    // Motion Profile Configuration
    double MAX_VELOCITY = 2.0; // TODO: Set actual maximum
    double MAX_ACCELERATION = 2.0; // TODO: Set actual maximum
    double TARGET_VELOCITY = 0.1;

    // Feedforward Constants
    double KS = 0;
    double KG = 0.46;
    double KV = 8.98;
    double KA = 0.05;

    // Operational Parameters
    double MANUAL_SPEED = 0.2;
    double ELEVATOR_UP_DIR = 1.0;
    double ELEVATOR_DOWN_DIR = -ELEVATOR_UP_DIR;

    // Preset Positions (TODO: Set actual values)
    double L1_POS = 0.0;
    double L2_POS = 0.0;
    double L3_POS = 0.0;
    double L4_POS = 0.0;
    int CORAL_INTAKE_POS = 0;
    int DRIVE_POS = 0;
    int MAX_SAFE_POS = 0;
  }

  interface Wrist {
    // Motor Configurations
    int PIVOT_MOTOR_ID = 53;

    // Encoder Configuration
    interface Encoder {
      int PORT = 1;
      int FULL_RANGE = 1024;
      int EXPECTED_ZERO = 0;
      double ABSOLUTE_OFFSET = 0.0; // TODO: Calibrate
    }

    // Roller Configuration
    interface Roller {
      int MOTOR_ID = 54;
      double CORAL_INTAKE_SPEED = 0;
      double CORAL_OUTTAKE_SPEED = 0;

      double ALGAE_INTAKE_SPEED = 0;
      double ALGAE_OUTTAKE_SPEED = 0;
    }

    // PID Configuration
    double PID_POSITION_TOLERANCE = 0.1;
    double PID_VELOCITY_TOLERANCE = 1;
    double KP = 0.1;
    double KI = 0.0;
    double KD = 0.0;

    // Motion Profile Configuration
    double MAX_VELOCITY = 2.0; // TODO: Set actual maximum
    double MAX_ACCELERATION = 2.0; // TODO: Set actual maximum

    // Feedforward Constants
    double KS = 0;
    double KG = 1.01;
    double KV = 0.81;
    double KA = 0.03;

    // Safety Limits
    double MAX_SAFE_ANGLE = 0;
    double MIN_SAFE_ANGLE = 0;

    // Preset Positions (TODO: Set actual values)
    int L1_ANGLE = 0;
    int L2_ANGLE = 0;
    int L3_ANGLE = 0;
    int L4_ANGLE = 0;
    int CORAL_INTAKE_ANGLE = 0;
    int BARGER_POSITION = 0;
    int DRIVE_POSITION = 0;

    // Game Piece Switching Angles
    int CORAL_MAX_ANGLE = 270;
    int CORAL_MIN_ANGLE = 20;
  }

  interface Swerve {
    interface Drive {
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
    double ROBOT_MASS_KG = 74.088;
    double ROBOT_MOI = 6.883;
    double WHEEL_COF = 1.2;

    // PID Constants
    interface PID {
      double DRIVE_KP = 5.0;
      double DRIVE_KI = 0.0;
      double DRIVE_KD = 0.0;

      double TURN_KP = 5.0;
      double TURN_KI = 0.0;
      double TURN_KD = 0.0;
    }
  }
}
