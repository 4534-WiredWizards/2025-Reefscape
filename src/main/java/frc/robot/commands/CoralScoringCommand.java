package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ScoringQueueSubsystem.ScoringHeight;
import frc.robot.subsystems.ScoringQueueSubsystem.ScoringSide;

public class CoralScoringCommand extends SequentialCommandGroup {
  private final ScoringSide side;
  private final ScoringHeight height;

  public CoralScoringCommand(ScoringSide side, ScoringHeight height) {
    super(
        new InstantCommand(() -> System.out.println("STARTING CoralScoringCommand")),
        new WaitCommand(0.1), // Small delay before marking completion
        new InstantCommand(() -> System.out.println("Scored Coral: " + side + " " + height)),
        new WaitCommand(0.1),
        new InstantCommand(() -> System.out.println("FINISHED CoralScoringCommand")));
    this.side = side;
    this.height = height;
  }

  public ScoringSide getSide() {
    return side;
  }

  public ScoringHeight getHeight() {
    return height;
  }
}
