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
    // Motor IDs
    public static final int LEFT_MOTOR_ID = 10; // Adjust as needed
    public static final int RIGHT_MOTOR_ID = 11; // Adjust as needed

    // Elevator Position Limits
    public static final double MAX_SAFE_POS = 50.0; // Maximum safe position (adjust as needed)
    public static final double MIN_SAFE_POS = 2.0; // Minimum safe position (adjust as needed)

    // Motion Magic Constants
    public static final double CRUISE_VELOCITY = 40.0; // In rotations/sec (~39.58 in/s)
    public static final double MAX_ACCELERATION = 80.0; // In rotations/sec²
    public static final double JERK = 800.0; // In rotations/sec³

    // Voltage Limits
    public static final double PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double PEAK_REVERSE_VOLTAGE = -12.0;
    public static final double ZEROING_VOLTAGE = 2.0;

    // PID and Feedforward Constants
    public static final double KP = 2.5; // Proportional gain
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.1; // Derivative gain

    // Feedforward Constants
    public static final double KS = 0.25; // Static friction compensation
    public static final double KV = 1.0; // Velocity feedforward
    public static final double KA = 0.1; // Acceleration feedforward
    public static final double KG = 0.25; // Gravity feedforward

    // Physical Constants
    public static final double PULLEY_DIAMETER = 120.0 / 25.4; // 120mm -> ~4.724 inches
    public static final double GEAR_RATIO = 15.0; // Verified 15:1 ratio

    // Conversion factors
    // Conversion factors (recalculated based on new measurements)
    public static final double ROTATIONS_TO_INCHES =
        (Math.PI * PULLEY_DIAMETER) / GEAR_RATIO; // ~0.9896 in/rot
    public static final double INCHES_TO_ROTATIONS = 1.0 / ROTATIONS_TO_INCHES; // ~1.0105 rot/in

    // Tolerance and Threshold Values
    public static final double POSITION_TOLERANCE = 1.0 * INCHES_TO_ROTATIONS; // ~1 inch tolerance
    public static final double STALL_POSITION_THRESHOLD =
        0.5 * INCHES_TO_ROTATIONS; // ~0.5 in movement
    // public static final double STALL_CURRENT_THRESHOLD = 25.0; // Current threshold for stall
    // detection
    // public static final double STALL_POSITION_THRESHOLD = 0.01; // Position change threshold for
    // stall detection

    // Elevator Stop Positions
    // Define preset positions that the elevator can move to
    public static final double POSITION_GROUND = 0.0;
    public static final double POSITION_L2 = 10.0;
    public static final double POSITION_L3 = 25.0;
    public static final double POSITION_L4 = 203;
    public static final double POSITION_BARGE = 203;

    // Manual Control Constants
    public static final double MANUAL_SPEED = 0.5; // Speed for manual control
    public static final int DOWN_DIRECTION = -1; // Direction for manual control
  }

  interface Wrist {
    // Motor and Encoder Configurations
    int PIVOT_MOTOR_ID = 53;
    double SPEED_SCALAR = 0.8;
    double GEAR_RATIO = 125.0;

    interface Encoder {
      int PORT = 1;
      int FULL_RANGE = 1024;
      int EXPECTED_ZERO = 0;
      double ABSOLUTE_OFFSET = 0.0; // TODO: Calibrate
    }

    // PID and Feedforward Configuration
    double PID_POSITION_TOLERANCE = 1 / 360;
    double PID_VELOCITY_TOLERANCE = 1;
    double KP = 0.012;
    double KI = 0.0005;
    double KD = 0.0001;

    // Feedforward Constants
    double KS = 0.25;
    double KG = 0.15;
    double KV = 2.25;
    double KA = 0.01;

    // Motion Profile Configuration
    double MAX_VELOCITY = 410;
    double MAX_ACCELERATION = 350;

    // Safety Limits
    double MAX_SAFE_ANGLE = 0 / 360.0; // 360; // Convert degrees to rotations
    double MIN_SAFE_ANGLE = -203 / 360.0;

    double MIN_CLEAR_ELEVATOR_ANGLE = 115;
    double TRUE_ZERO = 128;

    // Operational Parameters
    double STALL_VELOCITY_THRESHOLD = 0.1;
    double STALL_CURRENT_THRESHOLD = 30;

    // Preset Positions and Angles
    double L1_ANGLE = 128; // / 360;
    double L2_ANGLE = 105; // 360;
    double L3_ANGLE = 105; // / 360;
    double L4_ANGLE = 57; // / 360;
    int CORAL_INTAKE_ANGLE = 98;
    int BARGE_POSITION = 165;
    double PROCESSOR_POSITION = 165;
    int DRIVE_POSITION = 0;
    double SAFE_WRIST_POSITION = 100;

    // Game Piece Switching Angles
    int CORAL_MAX_ANGLE = 30;

    // Roller Configuration
    interface Roller {
      int MOTOR_ID = 54;
      int FIRST_SENSOR_ID = 2;
      int SECOND_SENSOR_ID = 3;

      double CORAL_INTAKE_SPEED = .1;
      double AFTER_FIRST_SENSOR_CORAL_SPEED = -.02;
      double CORAL_OUTTAKE_SPEED = -.3;
      double ALGAE_INTAKE_SPEED = .3;
      double ALGAE_OUTTAKE_SPEED = -.3;
    }
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
