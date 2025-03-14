package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.IO.Driver;
import frc.robot.Constants.IO.Operator;
import frc.robot.Constants.Wrist;
import frc.robot.commands.Climb.SimpleMoveClimb;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPath;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.SimpleMoveElevator;
import frc.robot.commands.ManualPoseSetter;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.RunCoralIntake;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.Wrist.SimpleMoveWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringQueueSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Vision vision;
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final WristSubsystem m_Wrist = new WristSubsystem(m_elevator);
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final ScoringQueueSubsystem m_scoringQueue;

  // Controllers
  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final Joystick driverJoystick = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

private PathPlannerPath Z1R;
private PathPlannerPath Z1L;
private PathPlannerPath Z2R;
private PathPlannerPath Z2L;
private PathPlannerPath Z3R;
private PathPlannerPath Z3L;
private PathPlannerPath Z4R;
private PathPlannerPath Z4L;
private PathPlannerPath Z5R;
private PathPlannerPath Z5L;
private PathPlannerPath Z6R;
private PathPlannerPath Z6L;



  public double getWristAngle() {
    return m_Wrist.getAngle();
  }
     

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
        // Z1R = PathPlannerPath.fromPathFile("1R");
        Z1L = PathPlannerPath.fromPathFile("1L");
        // Z2R = PathPlannerPath.fromPathFile("2R");
        // Z2L = PathPlannerPath.fromPathFile("2L");
        // Z3R = PathPlannerPath.fromPathFile("3R");
        // Z3L = PathPlannerPath.fromPathFile("3L");
        // Z4R = PathPlannerPath.fromPathFile("4R");
        // Z4L = PathPlannerPath.fromPathFile("4L");
        // Z5R = PathPlannerPath.fromPathFile("5R");
        // Z5L = PathPlannerPath.fromPathFile("5L");
        // Z6R = PathPlannerPath.fromPathFile("6R");
        // Z6L = PathPlannerPath.fromPathFile("6L");
    } catch (FileVersionException | IOException | ParseException e) {
        Z1R = null;
        Z1L = null;
        Z2R = null;
        Z2L = null;
        Z3R = null;
        Z3L = null;
        Z4R = null;
        Z4L = null;
        Z5R = null;
        Z5L = null;
        Z6R = null;
        Z6L = null;
    }

    // Initialize subsystems based on runtime mode
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive,
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera0Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Initialize scoring queue subsystem
    m_scoringQueue = new ScoringQueueSubsystem(drive, m_elevator, m_Wrist, m_Intake);

    // Register named commands for Pathplanner
    registerNamedCommands();

    // Set up event triggers
    configureEventTriggers();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add SysId characterization routines to auto chooser
    configureSysIdCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Set up SmartDashboard commands and test buttons
    configureSmartDashboard();

    // Setup manual pose setter
    setupManualPoseSetter();
  }

  /** Register named commands for PathPlanner */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand(
        "Outake", new AdaptiveWrist(m_Intake, this::getWristAngle, false));

    // Complex WE-CoralIntake command for intake with elevator coordination
    NamedCommands.registerCommand(
        "WE-CoralIntake",
        new ConditionalCommand(
            // If the elevator is NOT at ground position, run the full sequence
            new SequentialCommandGroup(
                // Step 1: Clear the elevator
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                // Step 2: Move elevator down, prepare wrist, and run intake
                new ParallelCommandGroup(
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(
                            () -> m_elevator.getEncoderPosition() < Elevator.ELEVATOR_DANGER_LIMIT),
                        new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)))),
            // If the elevator is ALREADY at ground position, just run intake and set wrist
            new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false),
            // The condition: Check if elevator is NOT at ground position
            () -> !m_elevator.isAtPosition(Elevator.POSITION_GROUND)));

    // Register different level scoring commands
    NamedCommands.registerCommand(
        "WE-L1",
        new SequentialCommandGroup(
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, Elevator.POSITION_L1, m_Wrist, false),
                new SetWristPosition(m_Wrist, Wrist.L1_ANGLE, false))));

    NamedCommands.registerCommand(
        "WE-L2",
        new SequentialCommandGroup(
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist, false),
                new SetWristPosition(m_Wrist, Wrist.L2_ANGLE, false))));

    NamedCommands.registerCommand(
        "WE-L3",
        new SequentialCommandGroup(
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist, false),
                new SetWristPosition(m_Wrist, Wrist.L3_ANGLE, false))));

    NamedCommands.registerCommand(
        "WE-L4",
        new SequentialCommandGroup(
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist, false),
                new SetWristPosition(m_Wrist, Wrist.L4_ANGLE, false))));
  }

  /** Configure event triggers for PathPlanner */
  private void configureEventTriggers() {
    // Elevator level events
    new EventTrigger("Elevator L4")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist));
    new EventTrigger("Elevator L3")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist));
    new EventTrigger("Elevator L2")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L1, m_Wrist));
    new EventTrigger("Elevator L1")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist));
    new EventTrigger("Elevator Ground")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist));

    // Wrist position events
    new EventTrigger("Wrist Coral L4").whileTrue(new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));
    new EventTrigger("Wrist Coral L3").whileTrue(new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
    new EventTrigger("Wrist Coral L2").whileTrue(new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
    new EventTrigger("Wrist Coral L1").whileTrue(new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));

    // Intake/Outake events
    new EventTrigger("Outake").whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false));
    new EventTrigger("Intake").whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true));
  }

  /** Configure SysId characterization commands for auto chooser */
  private void configureSysIdCommands() {
    // Drive characterization commands
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Wrist characterization commands
    autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)",
        m_Wrist.dynamicSysIdCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)",
        m_Wrist.dynamicSysIdCommand(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)", m_Wrist.sysIdCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)", m_Wrist.sysIdCommand(SysIdRoutine.Direction.kReverse));
  }

  /** Setup manual pose setter functionality */
  private void setupManualPoseSetter() {
    // Initialize default values for manual pose setter
    SmartDashboard.putNumber("ManualPose/X", 3.0);
    SmartDashboard.putNumber("ManualPose/Y", 3.0);
    SmartDashboard.putNumber("ManualPose/Rotation", 0.0);
    SmartDashboard.putData("ManualPose/SetPose", new ManualPoseSetter(drive));
  }

  /** Configure SmartDashboard commands and test controls */
  private void configureSmartDashboard() {
    // Command to reset pose based on vision
    SmartDashboard.putData("Reset Pose", new InstantCommand(() -> vision.resetRobotPose()));

    // Wrist test commands
    SmartDashboard.putData(
        "TestWristCommand/Wrist L4", new SetWristPosition(m_Wrist, Wrist.L4_ANGLE, false));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L3", new SetWristPosition(m_Wrist, Wrist.L3_ANGLE, false));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L2", new SetWristPosition(m_Wrist, Wrist.L2_ANGLE, false));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L1", new SetWristPosition(m_Wrist, Wrist.L1_ANGLE, false));
    SmartDashboard.putData("TestWristCommand/Bottom", new SetWristPosition(m_Wrist, -203, false));
    SmartDashboard.putData(
        "TestWristCommand/Wrist Coral Intake",
        new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE));
    SmartDashboard.putData(
        "TestWristCommand/Wrist MIN_CLEAR_ELEVATOR_ANGLE",
        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false));
    SmartDashboard.putData(
        "TestWristCommand/Wrist Drive", new SetWristPosition(m_Wrist, Wrist.DRIVE_POSITION, false));

    // Elevator test commands
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator Barge",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE, m_Wrist));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L4",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L3",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L2",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L1",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist));

    // Drive to point test commands
    // SmartDashboard.putData(
    //     "TestDrive/Zone1/Left",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_1, ScoringSide.LEFT)));
    // SmartDashboard.putData(
    //     "TestDrive/Zone1/Right",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_1, ScoringSide.RIGHT)));
    // SmartDashboard.putData(
    //     "TestDrive/Zone2/Left",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_2, ScoringSide.LEFT)));
    // SmartDashboard.putData(
    //     "TestDrive/Zone2/Right",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_2, ScoringSide.RIGHT)));
    // SmartDashboard.putData(
    //     "TestDrive/Zone3/Left",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_3, ScoringSide.LEFT)));
    // SmartDashboard.putData(
    //     "TestDrive/Zone3/Right",
    //     new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_3, ScoringSide.RIGHT)));
    // SmartDashboard.putData(
    //     "TestDrive/CoralStation2",
    //     new DriveToPoint(drive, new Pose2d(1.183, 0.956, new Rotation2d(Math.toRadians(234)))));

    SmartDashboard.putData("TestDrive/Zone1/Right", new DriveToPath(drive, Z1L));
    
  }

  /** Configure button bindings for driver and operator controls */
  private void configureButtonBindings() {
    // Default command for drive - joystick control
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_Y_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_X_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_ROTATE_AXIS),
            () -> driverJoystick.getRawAxis(Driver.DRIVE_THROTTLE_AXIS),
            () -> driverJoystick.getRawButton(Driver.SLOW_MODE_TOGGLE),
            () -> driverJoystick.getRawButton(Driver.FIELD_RELATIVE_TOGGLE)));

    // Lock to 0° when lock angle button is held
    new JoystickButton(driverJoystick, Driver.LOCK_ANGLE_BUTTON)
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverJoystick.getY(),
                () -> -driverJoystick.getX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    new JoystickButton(driverJoystick, Driver.STOP_WITH_X_BUTTON)
        .toggleOnTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero gyro when reset button is pressed
    new JoystickButton(driverJoystick, Driver.ZERO_GYRO_BUTTON)
        .onTrue(Commands.runOnce(() -> drive.resetGyro()).ignoringDisable(true));

    // Elevator manual control
    operatorController
        .leftBumper()
        .whileTrue(
            new SimpleMoveElevator(
                m_Wrist, m_elevator, () -> Elevator.DOWN_DIRECTION * Elevator.MANUAL_SPEED));
    operatorController
        .rightBumper()
        .whileTrue(
            new SimpleMoveElevator(
                m_Wrist, m_elevator, () -> (-1 * Elevator.DOWN_DIRECTION * Elevator.MANUAL_SPEED)));

    // Intake/Outtake controls
    operatorController
        .leftTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true)); // Intake
    operatorController
        .rightTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false)); // Outtake

    // Configure POV buttons for operator presets
    configurePOVButtons();

    // Configure climb controls
    operatorController.y().whileTrue(new SimpleMoveClimb(m_climb, () -> -0.5));
    operatorController.a().whileTrue(new SimpleMoveClimb(m_climb, () -> .3));

    // Side-specific controls using right thumb axis
    configureThumbAxisTriggers();

    // Set default command for wrist
    m_Wrist.setDefaultCommand(new SimpleMoveWrist(m_Wrist, () -> operatorController.getLeftX()));
  }

  /** Configure POV (D-pad) buttons for operator */
  private void configurePOVButtons() {
    // Coral intake sequence (Down button)
    operatorController
        .povDown()
        .onTrue(
            new ConditionalCommand(
                // If the elevator is NOT at ground position, run the full sequence
                new SequentialCommandGroup(
                    // Step 1: Clear the elevator
                    new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                    // Step 2: Move elevator down, prepare wrist, and run intake
                    new ParallelDeadlineGroup(
                        new RunCoralIntake(m_Intake, true),
                        new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(
                                () ->
                                    m_elevator.getEncoderPosition()
                                        < Elevator.ELEVATOR_DANGER_LIMIT),
                            new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)))),
                // If the elevator is ALREADY at ground position, just run intake and set wrist
                new ParallelCommandGroup(
                    new RunCoralIntake(m_Intake, true),
                    new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)),
                // The condition: Check if elevator is NOT at ground position
                () -> !m_elevator.isAtPosition(Elevator.POSITION_GROUND)));

    // L2 scoring position (Left button)
    operatorController
        .povLeft()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist, false),
                    new SetWristPosition(m_Wrist, Wrist.L2_ANGLE, false))));

    // L3 scoring position (Right button)
    operatorController
        .povRight()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist, false),
                    new SetWristPosition(m_Wrist, Wrist.L3_ANGLE, false))));

    // L4 scoring position (Up button)
    operatorController
        .povUp()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist, false),
                    new SetWristPosition(m_Wrist, Wrist.L4_ANGLE, false))));
  }

  /** Configure thumb axis triggers for side-specific actions */
  private void configureThumbAxisTriggers() {
    // Left side Level 1 with thumb axis left + D-pad down
    Trigger leftLevel1 =
        operatorController
            .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
            .and(operatorController.povDown());

    leftLevel1.onTrue(
        Commands.runOnce(
            () ->
                new SequentialCommandGroup(
                    new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE),
                    new ParallelCommandGroup(
                        new SetElevatorPosition(m_elevator, Elevator.POSITION_L1, m_Wrist),
                        new SetWristPosition(m_Wrist, Wrist.L1_ANGLE)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Updates SmartDashboard values periodically */
  public void updateDashboard() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Display current pose information
    Pose2d currentPose = drive.getPose();
    SmartDashboard.putNumber("Pose/X", currentPose.getX());
    SmartDashboard.putNumber("Pose/Y", currentPose.getY());
    SmartDashboard.putNumber("Pose/Rotation", currentPose.getRotation().getDegrees());

    // Display current zone
    SmartDashboard.putNumber("Drive/CurrentZone", drive.getZone().ordinal() + 1);
  }
}
