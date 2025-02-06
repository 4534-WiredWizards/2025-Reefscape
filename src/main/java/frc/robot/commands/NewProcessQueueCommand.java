// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringQueueSubsystem;
import java.util.LinkedList;
import java.util.Queue;

public class NewProcessQueueCommand extends Command {
  private final ScoringQueueSubsystem queueSubsystem;
  private Queue<CoralScoringCommand> commandQueue;
  private CoralScoringCommand currentCommand;

  public NewProcessQueueCommand(ScoringQueueSubsystem queueSubsystem) {
    this.queueSubsystem = queueSubsystem;
    commandQueue = new LinkedList<>();
  }

  @Override
  public void initialize() {
    // Copy the queue and clear the original
    commandQueue = new LinkedList<>(queueSubsystem.getQueue());
    queueSubsystem.clearQueue();
    currentCommand = null;
    System.out.println("Starting queue processing with " + commandQueue.size() + " commands");
  }

  @Override
  public void execute() {
    if (currentCommand == null && !commandQueue.isEmpty()) {
      currentCommand = commandQueue.poll();
      currentCommand.schedule();
      System.out.println("Starting command: " + currentCommand);
    }

    if (currentCommand != null && currentCommand.isFinished()) {
      System.out.println("Completed command: " + currentCommand);
      currentCommand = null; // Reset here
    }
  }

  @Override
  public boolean isFinished() {
    boolean finished = commandQueue.isEmpty() && currentCommand == null;
    System.out.println(
        "isFinished: "
            + finished
            + ", Queue size: "
            + commandQueue.size()
            + ", Current command: "
            + (currentCommand != null));
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && currentCommand != null) {
      currentCommand.cancel();
      System.out.println("Queue processing interrupted");
    }
    currentCommand = null; // Ensure it gets cleared
    System.out.println("Queue processing completed");
  }
}
