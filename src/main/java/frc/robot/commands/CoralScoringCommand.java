package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ScoringSide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;

public class CoralScoringCommand extends SequentialCommandGroup {

  private final ScoringSide side;
  private final Constants.ScoringHeight height;
  private final Drive drive;

  public CoralScoringCommand(
      Drive drive,
      ElevatorSubsystem elevator,
      WristSubsystem m_Wrist,
      IntakeSubsystem m_Intake,
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

              Pose2d targetPose =
                  new Pose2d(position.x(), position.y(), Rotation2d.fromDegrees(position.theta()));

              // Drive to the target position
              Logger.recordOutput("CoralScoringCommand/Status", "Driving to target position");

              addCommands(new DriveToPoint(drive, targetPose));

              // drive.driveToPoint(
              //     position.x(),
              //     position.y(),
              //     position
              //         .theta()); // FIXME: Needs to be a dynamic command with an isfinshed
              // command
            })
        // ,

        // Set elevator height (from enum configuration)
        // new InstantCommand(
        //         () -> {
        //           Logger.recordOutput("CoralScoringCommand/Status", "Setting elevator height");
        //           Logger.recordOutput(
        //               "CoralScoringCommand/ElevatorHeight", height.getElevatorPosition());
        //         })
        //     .andThen(new SetElevatorPosition(elevator, height.getElevatorPosition())),

        // // Use a DeadlineGroup to run AdaptiveWrist as the deadline
        // new ParallelDeadlineGroup(
        //     // Add a 0.5-second delay before starting AdaptiveWrist
        //     new WaitCommand(0.5)
        //         .andThen(
        //             new AdaptiveWrist(m_Intake, () -> m_Wrist.getAngle(), false).withTimeout(2)),
        //     new InstantCommand(
        //         () -> {
        //           Logger.recordOutput("CoralScoringCommand/Status", "Setting wrist angle");
        //           Logger.recordOutput("CoralScoringCommand/WristAngle", height.getWristAngle());
        //         }),
        //     new SetWristPosition(m_Wrist, height.getWristAngle(), false)),

        // // Return to drive position
        // new InstantCommand(
        //         () -> {
        //           Logger.recordOutput("CoralScoringCommand/Status", "Returning to drive
        // position");
        //         })
        //     .andThen(new SetElevatorPosition(elevator, Elevator.POSITION_GROUND)),

        // // Log the end of the command
        // new InstantCommand(
        //     () -> {
        //       Logger.recordOutput("CoralScoringCommand/Status", "Finished");
        //     })
        );
  }
}
