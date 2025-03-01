// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralIntake extends Command {
  /** Creates a new RunIntake. */
  private final IntakeSubsystem intakeSubsystem;

  private final boolean autoStop;
  private boolean coralCentered = false;
  private final Timer timer;
  private boolean secondSensorActivated = false;

  public RunCoralIntake(IntakeSubsystem intakeSubsystem, boolean autoStop) {
    this.intakeSubsystem = intakeSubsystem;
    this.autoStop = autoStop;
    timer = new Timer();
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralCentered = false;
    secondSensorActivated = false;
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    secondSensorActivated = intakeSubsystem.getSecondSensor();
    // If first sensor hasn't been activated yet, run at intake speed
    if (!secondSensorActivated) {
      intakeSubsystem.moveRoller(Wrist.Roller.CORAL_INTAKE_SPEED);
      System.out.println("second sensor not activated");}

      // Check if first sensor is now activated
      
      // If 0.5 seconds have passed, stop the motor
      if (secondSensorActivated) {
        System.out.println("Stopping roller");
        intakeSubsystem.stopRoller();
        coralCentered = true;
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (interrupted) {
      intakeSubsystem.stopRoller();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (autoStop && coralCentered) {
      return true;
    }
    return false;
  }
}
