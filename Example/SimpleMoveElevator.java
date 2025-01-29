import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class SimpleMoveElevator extends CommandBase {

    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;

    public SimpleMoveElevator(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Stop the PID controller when manual control starts
        elevator.stop();
    }

    @Override
    public void execute() {
        // Move the elevator manually based on the input speed
        double speed = speedSupplier.getAsDouble();
        elevator.moveManual(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator when the command ends
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
}