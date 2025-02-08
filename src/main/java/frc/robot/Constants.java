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

package frc.robot;

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

  // LED constants
  interface LEDConstants {
    int LED_ID = 26;
    double brightnessScalar = 0.4;
    int timeoutMs = 100;
    int CANdleLEDstartIndex = 0;
    int CANdleLEDSegmentSize = 8;
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
    double KG = 2.28;
    double KV = 3.07;
    double KA = 0.41;

    // Operational Parameters
    double MANUAL_SPEED = 0.2;
    double ELEVATOR_UP_DIR = 1.0;
    double ELEVATOR_DOWN_DIR = -ELEVATOR_UP_DIR;

    // Preset Positions (TODO: Set actual values)
    int TO_L1 = 0;
    int TO_L2 = 0;
    int TO_L3 = 0;
    int TO_L4 = 0;
    int CORAL_INTAKE_POSITION = 0;
    int DRIVE_POSITION = 0;
    int MAX_SAFE_HEIGHT = 0;
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
    double KG = 2.28;
    double KV = 3.07;
    double KA = 0.41;

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
    // This is
    int CORAL_MAX_ANGLE = 270;
    int CORAL_MIN_ANGLE = 20;
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
      int DRIVE_ROTATE_AXIS = 4;

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
