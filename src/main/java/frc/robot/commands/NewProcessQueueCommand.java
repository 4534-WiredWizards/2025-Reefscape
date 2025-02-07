// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringQueueSubsystem;

public class NewProcessQueueCommand extends SequentialCommandGroup {
  public NewProcessQueueCommand(ScoringQueueSubsystem queueSubsystem) {
    // Copy the queue and clear the original
    Queue<CoralScoringCommand> queue = new LinkedList<>(queueSubsystem.getQueue());
    queueSubsystem.clearQueue();

    // Add all commands to the sequential group
    for (CoralScoringCommand cmd : queue) {
      addCommands(cmd);
    }

    // System.out.println("Processing queue with " + queue.size() + " commands");
  }
}
