package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringHeight;
import frc.robot.Constants.ScoringSide;
import frc.robot.commands.CoralScoringCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.Queue;

public class ScoringQueueSubsystem extends SubsystemBase {
  private final Queue<CoralScoringCommand> commandQueue = new LinkedList<>();
  private final Drive drive;
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final IntakeSubsystem intake;

  public ScoringQueueSubsystem(
      Drive drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;
  }

  /**
   * Adds a scoring command to the queue.
   *
   * @param side The side (LEFT or RIGHT) to score on.
   * @param height The height (L1, L2, L3, L4) to score at.
   */
  public void addScoringCommand(ScoringSide side, ScoringHeight height) {

    // Create and add the scoring command to the queue
    commandQueue.add(new CoralScoringCommand(drive, elevator, wrist, intake, side, height));
  }

  /** Returns a copy of the current command queue. */
  public Queue<CoralScoringCommand> getQueue() {
    return new LinkedList<>(commandQueue);
  }

  /** Clears all commands from the queue. */
  public void clearQueue() {
    commandQueue.clear();
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with queue information
    SmartDashboard.putNumber("ScoringQueue/Size", commandQueue.size());
  }
}
