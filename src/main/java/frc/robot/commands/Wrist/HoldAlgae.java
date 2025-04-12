package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * A command that runs the algae intake at reduced power to hold the algae in place
 * until the command is interrupted.
 */
public class HoldAlgae extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double holdPowerPercentage;
  private final boolean useTorqueControl;
  
  // Constants for torque control
  private static final double BASELINE_HOLD_CURRENT = 10.0; // Amps
  
  /**
   * Creates a new HoldAlgae command with the default power level.
   *
   * @param intakeSubsystem The intake subsystem
   */
  public HoldAlgae(IntakeSubsystem intakeSubsystem) {
    this(intakeSubsystem, 0.5, false); // Default to 50% of normal intake power
  }

  /**
   * Creates a new HoldAlgae command with a specified power level.
   *
   * @param intakeSubsystem The intake subsystem
   * @param powerPercentage Percentage of normal intake power (0.0 to 1.0)
   */
  public HoldAlgae(IntakeSubsystem intakeSubsystem, double powerPercentage) {
    this(intakeSubsystem, powerPercentage, false); // Default to speed control
  }
  
  /**
   * Creates a new HoldAlgae command with a specified power level and control mode.
   *
   * @param intakeSubsystem The intake subsystem
   * @param powerPercentage Percentage of normal intake power/torque (0.0 to 1.0)
   * @param useTorqueControl Whether to use torque control instead of speed control
   */
  public HoldAlgae(IntakeSubsystem intakeSubsystem, double powerPercentage, boolean useTorqueControl) {
    this.intakeSubsystem = intakeSubsystem;
    this.holdPowerPercentage = Math.max(0.0, Math.min(1.0, powerPercentage));
    this.useTorqueControl = useTorqueControl;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("HoldAlgae initialized" + (useTorqueControl ? " with torque control" : ""));
    
    if (useTorqueControl) {
      // Use torque control mode
      double targetCurrent = BASELINE_HOLD_CURRENT * holdPowerPercentage;
      intakeSubsystem.setTorque(targetCurrent);
      Logger.recordOutput("Intake/Status/HoldingAlgae", true);
      Logger.recordOutput("Intake/Status/HoldTargetCurrent", targetCurrent);
      Logger.recordOutput("Intake/Status/UsingTorqueControl", true);
    } else {
      // Use traditional speed control
      double speed = Wrist.Roller.ALGAE_INTAKE_SPEED * holdPowerPercentage;
      intakeSubsystem.moveRoller(speed);
      Logger.recordOutput("Intake/Status/HoldingAlgae", true);
      Logger.recordOutput("Intake/Status/HoldPower", speed);
      Logger.recordOutput("Intake/Status/UsingTorqueControl", false);
    }
  }

  @Override
  public void execute() {
    if (useTorqueControl) {
      // Update torque setting in case current has changed
      double targetCurrent = BASELINE_HOLD_CURRENT * holdPowerPercentage;
      intakeSubsystem.setTorque(targetCurrent);
      
      // Log actual current vs target for debugging
      double actualCurrent = intakeSubsystem.getCurrentAmps();
      Logger.recordOutput("Intake/Status/CurrentDelta", targetCurrent - actualCurrent);
    } else {
      // Continue applying speed setting
      double speed = Wrist.Roller.ALGAE_INTAKE_SPEED * holdPowerPercentage;
      intakeSubsystem.moveRoller(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("HoldAlgae ended, interrupted: " + interrupted);
    intakeSubsystem.stopRoller();
    Logger.recordOutput("Intake/Status/HoldingAlgae", false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}