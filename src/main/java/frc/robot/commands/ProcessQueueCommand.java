package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringQueueSubsystem;
import java.util.Queue;

public class ProcessQueueCommand extends SequentialCommandGroup {
  public ProcessQueueCommand(ScoringQueueSubsystem queueSubsystem) {
    Queue<CoralScoringCommand> queue = queueSubsystem.getQueue();

    queueSubsystem.clearQueue();

    while (!queue.isEmpty()) {
      addCommands(queue.poll());
    }
  }
}
