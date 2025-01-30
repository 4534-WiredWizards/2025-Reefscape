// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

    // Test 123

    // Subsystems
    interface C_Elevator {
        double PIDPositionTolerance = 0.1;
        double PIDVelocityTolerance = 1;

        int leftMotorID = 51;
        int rightMotorID = 52;
        int BottomLimitSwitchID = 0;

        int toL1= 0;
        int toL2= 0;
        int toL3= 0;
        int toL4= 0;
        int coralIntakePosition= 0;
        int drivePosition= 0;
        int toBarge= 0;
        int climbGrabPosition= 0;
        int climbFinalPosition= 0;
        int climbGroundPosition= 0;
        int algae1Position= 0;
        int algae2Position= 0;
        int processorPosition= 0;
        int maxSafeUp= 0;
        double kS= 0;
        double kG= 2.28;
        double kV= 3.07;
        double kA= 0.41;
        double TargetVelocity= 2.0;
        double kP = 0.1;  // Proportional gain
        double kI = 0.0;  // Integral gain
        double kD = 0.0;  // Derivative gain

    }

    interface C_Wrist {

        interface Encoder {
            int port =1;
            int fullRange = 1024;
            int expectedZero = 0;
        }

     
        // Rollermotor for intake and scoring of game pieces
        interface Roller {
            int MotorID = 54;
            int SpeedCoralIn= 0;
            int SpeedCoralOut= 0;
            int SpeedAlgaeIn= 0;
            int SpeedAlgaeOut= 0;
        }
        



        // Pivot motor for wrist angle movment
        int pivotMotorID = 53;

        double kS= 0;
        double kG= 2.28;
        double kV= 3.07;
        double kA= 0.41;
        double TargetVelocity= 2.0;
        double kP = 0.1;  // Proportional gain
        double kI = 0.0;  // Integral gain
        double kD = 0.0;  // Derivative gain
        double AbsEncoderOffset = 0.0; // TODO
        double PIDPositionTolerance = 0.1;
        double PIDVelocityTolerance = 1;
        
    
        // Scoring Angles
        int L1= 0;
        int L2= 0;
        int L3= 0;
        int L4= 0;
        int CoralIntakePosition= 0;
        int BargePosition= 0;
        int Algae1= 0;
        int Algae2= 0;
        int AlgaeGround= 0;
        int DrivePosition= 0;
        
        // Game Piece Switching Angles
        // This is 
        int coralMaxAngle = 270;
        int coralMinAngle = 20;

        

    }


   

    interface C_Locations {

        int locReef= 0;
        int locBarge= 0;
        int locProcessor= 0;
        int locCoralStation= 0;
    }

}