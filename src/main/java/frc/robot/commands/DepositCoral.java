// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.C_Elevator;
import frc.robot.Constants.C_Locations;
import frc.robot.Constants.C_Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepositCoral extends SequentialCommandGroup {
  /** Creates a new DepositCoral. */
  public DepositCoral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveElevator(C_Elevator.toL4),
        new ChangeWristAngle(C_Wrist.wristL4),
        new AssistedMove(C_Locations.locReef),
        new RunIntake(C_Intake.intakeSpeedCoralOut),
        new MoveElevator(C_Elevator.drivePosition),
        new ChangeWristAngle(C_Wrist.wristCoralIntakePosition));
  }
}
