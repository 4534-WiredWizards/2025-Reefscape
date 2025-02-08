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
        System.out.println("Adding command: " + side + " " + height);
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
        // Update level status indicators
        updateLevelStatus();
    }

    private void updateLevelStatus() {
        boolean hasL1 = false;
        boolean[] leftLevels = new boolean[4];  // Index 0 unused (L1 handled separately)
        boolean[] rightLevels = new boolean[4];

        for (CoralScoringCommand cmd : commandQueue) {
            ScoringHeight height = cmd.getHeight();
            if (height == ScoringHeight.L1) {
                hasL1 = true;
                continue;
            }

            int levelIndex = height.ordinal() - 1;  // Convert L2-L4 to 1-3
            if (cmd.getSide() == ScoringSide.LEFT) {
                leftLevels[levelIndex] = true;
            } else {
                rightLevels[levelIndex] = true;
            }
        }

        // Update SmartDashboard entries
        SmartDashboard.putBoolean("Queued/L1", hasL1);
        SmartDashboard.putBoolean("Queued/Left/L2", leftLevels[0]);
        SmartDashboard.putBoolean("Queued/Left/L3", leftLevels[1]);
        SmartDashboard.putBoolean("Queued/Left/L4", leftLevels[2]);
        SmartDashboard.putBoolean("Queued/Right/L2", rightLevels[0]);
        SmartDashboard.putBoolean("Queued/Right/L3", rightLevels[1]);
        SmartDashboard.putBoolean("Queued/Right/L4", rightLevels[2]);
    }
}
