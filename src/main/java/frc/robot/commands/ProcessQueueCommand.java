package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringQueueSubsystem;
import java.util.Queue;

public class ProcessQueueCommand extends SequentialCommandGroup {
  public ProcessQueueCommand(ScoringQueueSubsystem queueSubsystem) {
    System.out.println("Processing queue 1");

    Queue<CoralScoringCommand> queue = queueSubsystem.getQueue();

    queueSubsystem.clearQueue();

    // System log that queue is being processed
    System.out.println("Processing queue 2");
    while (!queue.isEmpty()) {
      addCommands(queue.poll());
    }
  }
}
