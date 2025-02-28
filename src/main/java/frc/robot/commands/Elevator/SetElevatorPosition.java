package frc.robot.commands.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPosition extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_targetPosition;

    public SetElevatorPosition(ElevatorSubsystem elevator, double targetPosition) {
        this.m_elevator = elevator;
        this.m_targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Apply safety checks before moving
        double safePosition = Math.min(Elevator.MAX_SAFE_POS, 
            Math.max(Elevator.MIN_SAFE_POS, m_targetPosition));
        m_elevator.setPosition(safePosition);
    }

    @Override
    public void execute() {
        // Optional: Add progress logging
        double currentPosition = m_elevator.getEncoderPosition();
        Logger.recordOutput("Elevator/Command/Progress", 
            (currentPosition - Elevator.MIN_SAFE_POS) / 
            (m_targetPosition - Elevator.MIN_SAFE_POS) * 100);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_elevator.Stop();
            Logger.recordOutput("Elevator/Command/Interrupted", true);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when at position or if safety stop triggered
        return m_elevator.isAtPosition(m_targetPosition) 
            || m_elevator.isStalled();
    }
}