// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Misc;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.MoveElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoral extends SequentialCommandGroup {
  /** Creates a new IntakeCoral. */
  public IntakeCoral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new MoveElevator(C_Elevator.coralIntakePosition),
      new ChangeWristAngle(C_Wrist.wristCoralIntakePosition),
      new AssistedMove(C_Locations.locCoralStation),
      new RunIntake(C_Intake.intakeSpeedCoralIn),
      new ChangeWristAngle(C_Wrist.wristDrivePosition),
      new MoveElevator(C_Elevator.drivePosition)

    );
  }
}
