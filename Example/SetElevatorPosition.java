import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorPosition extends CommandBase {

    private final ElevatorSubsystem elevator;
    private final double targetPosition;

    public SetElevatorPosition(ElevatorSubsystem elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Set the desired position
        elevator.setSetpoint(targetPosition);
    }

    @Override
    public void execute() {
        // The PID controller will run in the subsystem's periodic method
    }

    @Override
    public boolean isFinished() {
        // Check if the elevator has reached the target position (within a tolerance)
        return Math.abs(elevator.getEncoderPosition() - targetPosition) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator motor
        elevator.stop();
    }
}