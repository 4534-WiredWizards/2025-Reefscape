package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.ScoringSide;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class CoralScoringCommand extends SequentialCommandGroup {

  private final ScoringSide side;
  private final Constants.ScoringHeight height;
  private final Drive drive;

  public CoralScoringCommand(
      Drive drive,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      ScoringSide side,
      Constants.ScoringHeight height) {
    this.drive = drive;
    this.side = side;
    this.height = height;

    addCommands(
        // Dynamically determine the zone at execution time
        new InstantCommand(
            () -> {
              Logger.recordOutput("CoralScoringCommand/Status", "Starting");
              Logger.recordOutput("CoralScoringCommand/Side", side.toString());
              Logger.recordOutput("CoralScoringCommand/Height", height.toString());

              // Get the current zone
              Constants.ReefZone reefZone = drive.getZone();

              // Get the target position
              Constants.ScoringPositions.ZonePosition position =
                  Constants.ScoringPositions.getZonePosition(reefZone, side);

              // Drive to the target position
              Logger.recordOutput("CoralScoringCommand/Status", "Driving to target position");
              drive.driveToPoint(position.x(), position.y(), position.theta());
            }),

        // Set elevator height (from enum configuration)
        new InstantCommand(
                () -> {
                  Logger.recordOutput("CoralScoringCommand/Status", "Setting elevator height");
                  Logger.recordOutput(
                      "CoralScoringCommand/ElevatorHeight", height.getElevatorPosition());
                })
            .andThen(new SetElevatorPosition(elevator, height.getElevatorPosition())),

        // Use a DeadlineGroup to run AdaptiveWrist as the deadline
        new ParallelDeadlineGroup(
            // Add a 0.5-second delay before starting AdaptiveWrist
            new WaitCommand(0.5).andThen(new AdaptiveWrist(wrist, false).withTimeout(2)),
            new InstantCommand(
                () -> {
                  Logger.recordOutput("CoralScoringCommand/Status", "Setting wrist angle");
                  Logger.recordOutput("CoralScoringCommand/WristAngle", height.getWristAngle());
                }),
            new SetWristPosition(wrist, height.getWristAngle(), false)),

        // Return to drive position
        new InstantCommand(
                () -> {
                  Logger.recordOutput("CoralScoringCommand/Status", "Returning to drive position");
                })
            .andThen(new SetElevatorPosition(elevator, Elevator.DRIVE_POS)),

        // Log the end of the command
        new InstantCommand(
            () -> {
              Logger.recordOutput("CoralScoringCommand/Status", "Finished");
            }));
  }
}
