package frc.robot.commands;

import java.util.Queue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringQueueSubsystem;

public class ProcessQueueCommand extends SequentialCommandGroup {
  public ProcessQueueCommand(ScoringQueueSubsystem queueSubsystem) {
    Queue<CoralScoringCommand> queue = queueSubsystem.getQueue();
    
    queueSubsystem.clearQueue();
    
    while (!queue.isEmpty()) {
      addCommands(queue.poll());
    }
  }
}
