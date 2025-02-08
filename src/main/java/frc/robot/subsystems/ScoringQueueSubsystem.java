// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CoralScoringCommand;
import java.util.LinkedList;
import java.util.Queue;

public class ScoringQueueSubsystem extends SubsystemBase {
  private final Queue<CoralScoringCommand> commandQueue = new LinkedList<>();

  public enum ScoringSide {
    LEFT,
    RIGHT
  }

  public enum ScoringHeight {
    L1,
    L2,
    L3,
    L4
  }

  private boolean QueuedL1 = false;
  private boolean QueuedLeftL2 = false;
  private boolean QueuedLeftL3 = false;
  private boolean QueuedLeftL4 = false;
  private boolean QueuedRightL2 = false;
  private boolean QueuedRightL3 = false;
  private boolean QueuedRightL4 = false;

  public void addScoringCommand(ScoringSide side, ScoringHeight height) {

    // Systsem log for scoring side and height
    System.out.println("Scoring command added to queue: " + side + " " + height);

    if (side == ScoringSide.LEFT) {
      if (height == ScoringHeight.L2) {
        QueuedLeftL2 = true;
      } else if (height == ScoringHeight.L3) {
        QueuedLeftL3 = true;
      } else if (height == ScoringHeight.L4) {
        QueuedLeftL4 = true;
      }
    }
    if (height == ScoringHeight.L1) {
      QueuedL1 = true;
    } else {
      if (height == ScoringHeight.L2) {
        QueuedRightL2 = true;
      } else if (height == ScoringHeight.L3) {
        QueuedRightL3 = true;
      } else if (height == ScoringHeight.L4) {
        QueuedRightL4 = true;
      }
    }

    // // Clear the queue before adding a new command
    // commandQueue.clear();

    // // Log command added in logs
    // // System.out.println("Scoring command added to queue");
    // commandQueue.add(new CoralScoringCommand(side, height));
  }

  public Queue<CoralScoringCommand> getQueue() {
    return new LinkedList<>(commandQueue);
  }

  public void clearQueue() {
    commandQueue.clear();
    QueuedL1 = false;
    QueuedLeftL2 = false;
    QueuedLeftL3 = false;
    QueuedLeftL4 = false;

    QueuedRightL2 = false;
    QueuedRightL3 = false;
    QueuedRightL4 = false;
  }

  @Override
  public void periodic() {
    // Convert queue entries to human-readable strings
    String[] queueEntries =
        commandQueue.stream()
            .map(command -> command.toString()) // Assuming CoralScoringCommand has toString()
            .toArray(String[]::new);

    SmartDashboard.putStringArray("Scoring Queue", queueEntries);

    SmartDashboard.putBoolean("QueuedL1", QueuedL1);
    SmartDashboard.putBoolean("QueuedLeftL2", QueuedLeftL2);
    SmartDashboard.putBoolean("QueuedLeftL3", QueuedLeftL3);
    SmartDashboard.putBoolean("QueuedLeftL4", QueuedLeftL4);

    SmartDashboard.putBoolean("QueuedRightL2", QueuedRightL2);
    SmartDashboard.putBoolean("QueuedRightL3", QueuedRightL3);
    SmartDashboard.putBoolean("QueuedRightL4", QueuedRightL4);
  }
}
