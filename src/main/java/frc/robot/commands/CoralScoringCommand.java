// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringQueueSubsystem.ScoringHeight;
import frc.robot.subsystems.ScoringQueueSubsystem.ScoringSide;

public class CoralScoringCommand extends SequentialCommandGroup {

  private final ScoringSide side;
  private final ScoringHeight height;

  public CoralScoringCommand(ScoringSide side, ScoringHeight height) {
    this.side = side;
    this.height = height;
    addCommands(
        // System out for now
        new InstantCommand(() -> System.out.println("Scored Coral: " + side + " " + height)
    )
    // new AutoAlignCommand(side)
    // new ElevatorPositionCommand(height),
    // new ScoreElementCommand(),
    // new ReturnToHomeCommand()
    );
  }

  @Override
  public String toString() {
    return side.name() + " " + height.name();
  }
}
