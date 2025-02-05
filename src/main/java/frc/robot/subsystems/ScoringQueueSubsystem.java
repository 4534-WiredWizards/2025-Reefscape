// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CoralScoringCommand;

public class ScoringQueueSubsystem extends SubsystemBase {
  private final Queue<CoralScoringCommand> commandQueue = new LinkedList<>();

  public enum ScoringSide { LEFT, RIGHT }
  public enum ScoringHeight { L1, L2, L3, L4 }

  public void addScoringCommand(ScoringSide side, ScoringHeight height) {
      commandQueue.add(new CoralScoringCommand(side, height));
  }

  public Queue<CoralScoringCommand> getQueue() {
      return new LinkedList<>(commandQueue);
  }

  public void clearQueue() {
      commandQueue.clear();
  }

  @Override
  public void periodic() {
      // Convert queue entries to human-readable strings
      String[] queueEntries = commandQueue.stream()
          .map(command -> command.toString()) // Assuming CoralScoringCommand has toString()
          .toArray(String[]::new);
      
      SmartDashboard.putStringArray("Scoring Queue", queueEntries);
  }
}
