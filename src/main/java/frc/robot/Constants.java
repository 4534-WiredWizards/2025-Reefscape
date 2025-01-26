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

    // Subsystems
    interface C_Elevator {
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

    }

    interface C_Wrist {
    
        int wristMotorID = 53;

        int wristL1= 0;
        int wristL2= 0;
        int wristL3= 0;
        int wristL4= 0;
        int wristCoralIntakePosition= 0;
        int wristBargePosition= 0;
        int wristAlgae1= 0;
        int wristAlgae2= 0;
        int wristAlgaeGround= 0;
        int wristDrivePosition= 0;
        
    }


    interface C_Intake {
        int intakeMotorID = 54;

        int intakeSpeedCoralIn= 0;
        int intakeSpeedCoralOut= 0;
        int intakeSpeedAlgaeIn= 0;
        int intakeSpeedAlgaeOut= 0;
    }

    interface C_Locations {

        int locReef= 0;
        int locBarge= 0;
        int locProcessor= 0;
        int locCoralStation= 0;
    }

}