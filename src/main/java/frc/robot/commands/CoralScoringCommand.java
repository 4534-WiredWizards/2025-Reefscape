package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ReefZone;
import frc.robot.Constants.ScoringSide;
import frc.robot.Constants.Wrist;
import frc.robot.RobotContainer;
import frc.robot.commands.Wrist.RunCoralOutake;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class CoralScoringCommand extends SequentialCommandGroup {

  private final ScoringSide side;
  private final Constants.ScoringHeight height;
  private final Drive drive;
  public RobotContainer robotContainer;

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
              ReefZone currentZone = drive.getZone();
              Logger.recordOutput("AutoScoring/ExecutionZone", currentZone.toString());

              // Get the path for the current zone and side
              PathPlannerPath path = robotContainer.getPathForZoneAndSide(currentZone, side);

              // Create and schedule a command to follow that specific path
              Command command =
                  new SequentialCommandGroup(
                      new InstantCommand(
                          () ->
                              Logger.recordOutput(
                                  "AutoScoring/StartingPath",
                                  currentZone.toString() + "-" + side.toString())),
                      new DriveToPath(drive, path),
                      // Rest of your scoring sequence...
                      new RunCoralOutake(m_Intake),
                      new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true));
              command.schedule();
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
