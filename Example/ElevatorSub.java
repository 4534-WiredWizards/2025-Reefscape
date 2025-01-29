import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {

    // Motor controller for the elevator
    private final CANSparkFlex elevatorMotor;

    // Encoder for the elevator
    private final RelativeEncoder elevatorEncoder;

    // PID controller for the elevator
    private final PIDController pidController;

    // Constants for PID tuning
    private static final double kP = 0.1;  // Proportional gain
    private static final double kI = 0.0;  // Integral gain
    private static final double kD = 0.0;  // Derivative gain

    // Feedforward constant (if needed)
    private static final double kF = 0.0;  // Feedforward gain

    // Elevator setpoint (desired position)
    private double setpoint;

    public ElevatorSubsystem() {
        // Initialize motor controller
        elevatorMotor = new CANSparkFlex(1, MotorType.kBrushless);  // Replace with the correct CAN ID

        // Initialize encoder
        elevatorEncoder = elevatorMotor.getEncoder();

        // Configure encoder (adjust these values based on your setup)
        elevatorEncoder.setPositionConversionFactor(1.0);  // Set conversion factor (e.g., inches per rotation)

        // Initialize PID controller
        pidController = new PIDController(kP, kI, kD);

        // Set the initial setpoint to the current position
        setpoint = getEncoderPosition();
    }

    // Get the current encoder position
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    // Set the desired setpoint (elevator position)
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Run the PID controller to move the elevator to the setpoint
    public void runPID() {
        // Calculate the PID output
        double pidOutput = pidController.calculate(getEncoderPosition(), setpoint);

        // Add feedforward (if needed)
        double feedforward = kF * setpoint;

        // Set the motor output
        elevatorMotor.set(pidOutput + feedforward);
    }

    // Move the elevator manually (for manual driving)
    public void moveManual(double speed) {
        // Disable PID control and set the motor speed directly
        elevatorMotor.set(speed);
    }

    // Stop the elevator motor
    public void stop() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        runPID();
    }
}