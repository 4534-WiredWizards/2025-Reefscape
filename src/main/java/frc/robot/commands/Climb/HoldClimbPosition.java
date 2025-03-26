package frc.robot.commands.Climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import org.littletonrobotics.junction.Logger;

public class HoldClimbPosition extends Command {
  private final ClimbSubsystem m_climb;
  private final PIDController pidController;
  private double targetPosition;
  
  // PID constants - tune these for your specific hardware
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  
  // Position tolerance for the isFinished method (optional)
  private static final double POSITION_TOLERANCE = 0.5;
  
  // Maximum output to prevent excessive motor power
  private static final double MAX_OUTPUT = 0.5;

  /**
   * Creates a new HoldClimbPosition command.
   * 
   * @param m_climb The climb subsystem
   */
  public HoldClimbPosition(ClimbSubsystem m_climb) {
    this.m_climb = m_climb;
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(POSITION_TOLERANCE);
    
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {
    // Capture the current encoder position as the target
    targetPosition = m_climb.climbMotor.getEncoder().getPosition();
    
    // Reset the PID controller
    pidController.reset();
    
    Logger.recordOutput("Climb/HoldPosition/Target", targetPosition);
    Logger.recordOutput("Climb/HoldPosition/Active", true);
  }

  @Override
  public void execute() {
    // Get current position
    double currentPosition = m_climb.climbMotor.getEncoder().getPosition();
    
    // Calculate motor output using PID
    double output = pidController.calculate(currentPosition, targetPosition);
    
    // Limit the output to prevent excessive power
    output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));
    
    // Apply the calculated output to the motor
    m_climb.moveManual(output);
    
    // Log data for debugging
    Logger.recordOutput("Climb/HoldPosition/Current", currentPosition);
    Logger.recordOutput("Climb/HoldPosition/Error", targetPosition - currentPosition);
    Logger.recordOutput("Climb/HoldPosition/Output", output);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    m_climb.stop();
    Logger.recordOutput("Climb/HoldPosition/Active", false);
  }

  @Override
  public boolean isFinished() {
    // This command should run until interrupted
    return false;
  }
}