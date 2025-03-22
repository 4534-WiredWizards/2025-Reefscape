package frc.robot;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.IO.Driver;
import frc.robot.Constants.IO.Operator;
import frc.robot.Constants.ReefZone;
import frc.robot.Constants.ScoringSide;
import frc.robot.Constants.Wrist;
import frc.robot.commands.Climb.SimpleMoveClimb;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPath;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.SimpleMoveElevator;
import frc.robot.commands.ManualPoseSetter;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.RunCoralIntake;
import frc.robot.commands.Wrist.RunCoralOutake;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.Wrist.SimpleMoveWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  public final WristSubsystem m_Wrist = new WristSubsystem(m_elevator);
  private final ClimbSubsystem m_climb = new ClimbSubsystem();

  // Controllers
  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final Joystick driverJoystick = new Joystick(1);

  // Requested Position
  private double targetElevatorPosition = Elevator.POSITION_L1;
  private double targetWristAngle = Wrist.L1_ANGLE;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Path Planning Paths
  public PathPlannerPath Z1R;
  public PathPlannerPath Z1L;
  public PathPlannerPath Z2R;
  public PathPlannerPath Z2L;
  public PathPlannerPath Z3R;
  public PathPlannerPath Z3L;
  public PathPlannerPath Z4R;
  public PathPlannerPath Z4L;
  public PathPlannerPath Z5R;
  public PathPlannerPath Z5L;
  public PathPlannerPath Z6R;
  public PathPlannerPath Z6L;

  public double getWristAngle() {
    return m_Wrist.getAngle();
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Load path planner paths
    loadPaths();

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

    // Register named commands for Pathplanner
    registerNamedCommands();

    // Set up event triggers
    configureEventTriggers();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();

    // Set up SmartDashboard commands and test buttons
    configureSmartDashboard();

    // Setup manual pose setter
    setupManualPoseSetter();
  }

  /** Loads all path planner paths used for autonomous and reef scoring */
  private void loadPaths() {
    try {
      System.out.println("\n[Path Loading] Loading paths...");
      Z1R = PathPlannerPath.fromPathFile("1R");
      Z1L = PathPlannerPath.fromPathFile("1L");
      Z2R = PathPlannerPath.fromPathFile("2R");
      Z2L = PathPlannerPath.fromPathFile("2L");
      Z3R = PathPlannerPath.fromPathFile("3R");
      Z3L = PathPlannerPath.fromPathFile("3L");
      Z4R = PathPlannerPath.fromPathFile("4R");
      Z4L = PathPlannerPath.fromPathFile("4L");
      Z5R = PathPlannerPath.fromPathFile("5R");
      Z5L = PathPlannerPath.fromPathFile("5L");
      Z6R = PathPlannerPath.fromPathFile("6R");
      Z6L = PathPlannerPath.fromPathFile("6L");
      System.out.println("-> All paths loaded successfully");
    } catch (FileVersionException | IOException | ParseException e) {
      System.err.println("!! ERROR LOADING PATHS !!");
      e.printStackTrace();
      // Handle null paths gracefully
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
  }

  /** Sets the target positions for the elevator and wrist */
  public void setTargetPositions(double elevatorPosition, double wristAngle) {
    this.targetElevatorPosition = elevatorPosition;
    this.targetWristAngle = wristAngle;
  }

  /** Gets the current target elevator position */
  public double getTargetElevatorPosition() {
    return targetElevatorPosition;
  }

  /** Gets the current target wrist angle */
  public double getTargetWristAngle() {
    return targetWristAngle;
  }

  /**
   * Gets the path to use based on the specified zone and side.
   *
   * @param zone The scoring zone (ZONE_1 through ZONE_6)
   * @param side The scoring side (LEFT or RIGHT)
   * @return The PathPlannerPath for the specified zone and side
   */
  public PathPlannerPath getPathForZoneAndSide(ReefZone zone, ScoringSide side) {
    System.out.println("\n[Path Selection] Determining path:");
    System.out.println("-> Current Zone: " + zone);
    System.out.println("-> Requested Side: " + side);
    System.out.println("-> Alliance: " + DriverStation.getAlliance().orElse(Alliance.Blue));

    PathPlannerPath path =
        switch (zone) {
          case ZONE_1 -> side == ScoringSide.RIGHT ? Z1R : Z1L;
          case ZONE_2 -> side == ScoringSide.RIGHT ? Z2R : Z2L;
          case ZONE_3 -> side == ScoringSide.RIGHT ? Z3R : Z3L;
          case ZONE_4 -> side == ScoringSide.RIGHT ? Z4R : Z4L;
          case ZONE_5 -> side == ScoringSide.RIGHT ? Z5R : Z5L;
          case ZONE_6 -> side == ScoringSide.RIGHT ? Z6R : Z6L;
          default -> null;
        };

    if (path != null) {
      System.out.println("-> Selected Path: " + path.name);
    } else {
      System.err.println("!! ERROR: No path found for zone " + zone + " side " + side);
    }

    return path;
  }

  /**
   * Creates a command to drive to a reef scoring position based on the current zone and specified
   * side. Includes interrupt capability using button press.
   *
   * @param side The scoring side (LEFT or RIGHT)
   * @return Command sequence for driving to the specified reef side
   */
  public Command driveToReefSide(ScoringSide side) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> vision.resetRobotPose()),
        // Create a new initial command that determines the path based on current zone
        new InstantCommand(
            () -> {
              // Get the current zone at execution time
              ReefZone currentZone = drive.getZone();
              Logger.recordOutput("DriveToReef/ExecutionZone", currentZone.toString());
              Logger.recordOutput("DriveToReef/RequestedSide", side.toString());

              // Get the path for the current zone and side
              PathPlannerPath path = getPathForZoneAndSide(currentZone, side);

              // Create a cancel trigger that can be used by the driver
              // This will be a new trigger for the driver joystick button 1 (trigger)
              Trigger cancelDriveTrigger =
                  new JoystickButton(driverJoystick, Driver.RightJoystick.TRIGGER);

              // Create and schedule a command to follow that specific path with cancel capability
              Command pathCommand =
              new SequentialCommandGroup(
                        new DriveToPath(drive, path)
                        .until(cancelDriveTrigger), // Will end when trigger is pressed
                        // Add rumble feedback when path completes
                        setOperatorRumble(0.7) // Use a moderate rumble intensity
                        ).finallyDo(
                          () -> {
                            // When the command ends (either by completion or cancellation),
                            // log the status and stop the drive
                            Logger.recordOutput(
                                "DriveToReef/Status",
                                "Path following ended - "
                                    + (cancelDriveTrigger.getAsBoolean()
                                        ? "Cancelled by driver"
                                        : "Completed successfully"));
                            drive.stop();

                            operatorController.setRumble(kBothRumble, 0.0);
                          });

              pathCommand.schedule();
              Logger.recordOutput("DriveToReef/Status", "Started driving to reef side");
            }));
  }

  /** Creates a sequence for moving to a scoring position */
  public Command createScoringSequence(double elevatorPosition, double wristAngle) {
    return new ConditionalCommand(
        // If coral is detected in intake (sensor is triggered)
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  System.out.println("Coral detected, running full sequence");
                  System.out.println("Requested Elevator Position: " + elevatorPosition);
                  System.out.println("Requested Wrist Angle: " + wristAngle);
                }),
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                // Use the sensor to determine when to stop waiting
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, elevatorPosition, m_Wrist, false),
                new SetWristPosition(m_Wrist, wristAngle, false))),
        // If no coral detected, just move to position without waiting for sensor
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  System.out.println("No coral detected, moving to position");
                  System.out.println("Requested Elevator Position: " + elevatorPosition);
                  System.out.println("Requested Wrist Angle: " + wristAngle);
                }),
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelCommandGroup(
                new SetElevatorPosition(m_elevator, elevatorPosition, m_Wrist, false),
                new SetWristPosition(m_Wrist, wristAngle, false))),
        // Condition: check if sensor detects coral
        () -> {
          boolean sensorTriggered = m_Intake.getFirstSensor();
          System.out.println(
              "Sensor check: " + (sensorTriggered ? "Coral detected" : "No coral detected"));
          return sensorTriggered;
        });
  }

  /** Creates a command sequence for elevator down and coral intake */
  public Command elevatorDownAndRunCoralIntake() {
        return new SequentialCommandGroup(
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
                                () -> m_elevator.getEncoderPosition() < Elevator.ELEVATOR_DANGER_LIMIT),
                            new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)))),
                // If the elevator is ALREADY at ground position, just run intake and set wrist
                new ParallelDeadlineGroup(
                    new RunCoralIntake(m_Intake, true),
                    new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)),
                // The condition: Check if elevator is NOT at ground position
                () -> !m_elevator.isAtPosition(0.3)),
            // Add rumble feedback after coral intake completes
            setOperatorRumble(0.6)
        );
    }

  /** Register named commands for PathPlanner */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand(
        "Outake", new AdaptiveWrist(m_Intake, this::getWristAngle, false));

    // Complex WE-CoralIntake command for intake with elevator coordination
    NamedCommands.registerCommand("WE-CoralIntake", elevatorDownAndRunCoralIntake());

    // Register different level scoring commands
    NamedCommands.registerCommand(
        "WE-L1", createScoringSequence(Elevator.POSITION_L1, Wrist.L1_ANGLE));
    NamedCommands.registerCommand(
        "WE-L2", createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));
    NamedCommands.registerCommand(
        "WE-L3", createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));
    NamedCommands.registerCommand("WE-L4", createScoringSequence(62.5, 110.0));

    // Elevator to zero position
    NamedCommands.registerCommand(
        "E-Zero", new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist, true));

    // Command to set wrist to safe position
    NamedCommands.registerCommand(
        "SetWristSafePosition",
        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true));

    NamedCommands.registerCommand(
        "SetTargetL4",
        new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4, Wrist.L4_ANGLE)));

    NamedCommands.registerCommand("RunCoralOutake", new RunCoralOutake(m_Intake).withTimeout(1.5));

    // Resetpose based on vision command
    NamedCommands.registerCommand(
        "ResetBotPose", new InstantCommand(() -> vision.resetRobotPose()).ignoringDisable(true));
  }

  /** Configure event triggers for PathPlanner */
  private void configureEventTriggers() {
    // Elevator level events
    new EventTrigger("MoveToRequestedPosition")
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                    new SetElevatorPosition(
                        m_elevator, this::getTargetElevatorPosition, m_Wrist, false),
                    new SetWristPosition(m_Wrist, this::getTargetWristAngle, false))));

    // Commands to move wrist and elevator to the scoring position
    new EventTrigger("WE-CoralIntake").onTrue(elevatorDownAndRunCoralIntake());
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
    SmartDashboard.putData(
        "Reset Pose", new InstantCommand(() -> vision.resetRobotPose()).ignoringDisable(true));

    SmartDashboard.putData("Run Coral Outake", new RunCoralOutake(m_Intake));

    // Wrist test commands
    SmartDashboard.putData("Wrist/L4", new SetWristPosition(m_Wrist, Wrist.L4_ANGLE, false));
    SmartDashboard.putData("Wrist/L3", new SetWristPosition(m_Wrist, Wrist.L3_ANGLE, false));
    SmartDashboard.putData("Wrist/L2", new SetWristPosition(m_Wrist, Wrist.L2_ANGLE, false));
    SmartDashboard.putData("Wrist/L1", new SetWristPosition(m_Wrist, Wrist.L1_ANGLE, false));
    SmartDashboard.putData(
        "Wrist/CoralIntake", new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE));
    SmartDashboard.putData(
        "Wrist/ClearElevator",
        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false));
    SmartDashboard.putData(
        "Wrist/Drive", new SetWristPosition(m_Wrist, Wrist.DRIVE_POSITION, false));

    // Elevator test commands
    SmartDashboard.putData(
        "Elevator/Barge", new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE, m_Wrist));
    SmartDashboard.putData(
        "Elevator/L4", new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist));
    SmartDashboard.putData(
        "Elevator/L3", new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist));
    SmartDashboard.putData(
        "Elevator/L2", new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist));
    SmartDashboard.putData(
        "Elevator/L1", new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist));

    // AutoScoring test
    // SmartDashboard.putData(
    //     "Score/Z1-l4-L",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> vision.resetRobotPose()),
    //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4, Wrist.L4_ANGLE)),
    //         new ParallelCommandGroup(
    //             new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
    //             new DriveToPath(drive, Z1L)),
    //         new RunCoralOutake(m_Intake),
    //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

    // Add Z1-L4 Right
    // SmartDashboard.putData(
    //     "Score/Z1-l4-R",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> vision.resetRobotPose()),
    //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4, Wrist.L4_ANGLE)),
    //         new ParallelCommandGroup(
    //             new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
    //             new DriveToPath(drive, Z1R)),
    //         new RunCoralOutake(m_Intake),
    //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

    // New method with cancellation capability
    SmartDashboard.putData("Score/AutoZone L", driveToReefSide(ScoringSide.LEFT));
    SmartDashboard.putData("Score/AutoZone R", driveToReefSide(ScoringSide.RIGHT));

    // Drive to path test commands
    // SmartDashboard.putData("Drive/Z1R", new DriveToPath(drive, Z1R));
    // SmartDashboard.putData("Drive/Z1L", new DriveToPath(drive, Z1L));
    // SmartDashboard.putData("Drive/Z2R", new DriveToPath(drive, Z2R));
    // SmartDashboard.putData("Drive/Z2L", new DriveToPath(drive, Z2L));
    // SmartDashboard.putData("Drive/Z3R", new DriveToPath(drive, Z3R));
    // SmartDashboard.putData("Drive/Z3L", new DriveToPath(drive, Z3L));
    // SmartDashboard.putData("Drive/Z4R", new DriveToPath(drive, Z4R));
    // SmartDashboard.putData("Drive/Z4L", new DriveToPath(drive, Z4L));
    // SmartDashboard.putData("Drive/Z5R", new DriveToPath(drive, Z5R));
    // SmartDashboard.putData("Drive/Z5L", new DriveToPath(drive, Z5L));
    // SmartDashboard.putData("Drive/Z6R", new DriveToPath(drive, Z6R));
    // SmartDashboard.putData("Drive/Z6L", new DriveToPath(drive, Z6L));
  }

// Method to set controller rumble for operator controller
public Command setOperatorRumble(double rumble) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> operatorController.setRumble(kBothRumble, rumble)),
                new WaitCommand(0.5),
                new InstantCommand(() -> operatorController.setRumble(kBothRumble, 0.0)));

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
            () -> driverJoystick.getRawButton(Driver.RightJoystick.RIGHT_THUMB_BUTTON),
            () -> driverJoystick.getRawButton(Driver.LeftThrottle.TOP_THUMB_BUTTON)));

    // Lock to 0° when lock angle button is held
    new JoystickButton(driverJoystick, Driver.LeftThrottle.BOTTOM_THUMB_BUTTON)
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverJoystick.getY(),
                () -> -driverJoystick.getX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    new JoystickButton(driverJoystick, Driver.LeftThrottle.MIDDLE_THUMB_BUTTON)
        .toggleOnTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero gyro when reset button is pressed
    new JoystickButton(driverJoystick, Driver.RightJoystick.STRIPED_CENTER_BUTTON)
        .onTrue(Commands.runOnce(() -> vision.resetRobotPose()).ignoringDisable(true));

    // Add driver joystick commands for reef side approachs
    new JoystickButton(driverJoystick, Driver.BASE_LEFT_BUTTON)
        .onTrue(driveToReefSide(ScoringSide.LEFT));
    new JoystickButton(driverJoystick, Driver.BASE_RIGHT_BUTTON)
        .onTrue(driveToReefSide(ScoringSide.RIGHT));

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

    operatorController
        .button(Operator.RESET_BOT_POSE_BUTTON)
        .onTrue(new InstantCommand(() -> vision.resetRobotPose()).ignoringDisable(true));
    operatorController.button(Operator.ZERO_ELEVATOR_BUTTON).onTrue(m_elevator.zeroCommand());

    // Configure POV buttons for operator presets
    configurePOVButtons();

    // Configure climb controls
    operatorController.y().whileTrue(new SimpleMoveClimb(m_climb, () -> -0.35)); // Wind - Climb up
    operatorController.x().whileTrue(new SimpleMoveClimb(m_climb, () -> 1)); // Unwind

    // A - Low algae
    operatorController
        .a()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelCommandGroup(
                    new SetElevatorPosition(
                        m_elevator, Elevator.POSITION_LOW_ALGAE, m_Wrist, false),
                    new SetWristPosition(m_Wrist, Wrist.ALGAE_INTAKE_ANGLE, false),
                    new AdaptiveWrist(m_Intake, this::getWristAngle, true))));

    // Test rumble command
    operatorController
        .start()
        .onTrue(new InstantCommand(() -> setOperatorRumble(1.0)))
        .onFalse(new InstantCommand(() -> setOperatorRumble(0.0)));

    // B - High algae
    operatorController
        .b()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelCommandGroup(
                    new SetElevatorPosition(
                        m_elevator, Elevator.POSITION_HIGH_ALGAE, m_Wrist, false),
                    new SetWristPosition(m_Wrist, Wrist.ALGAE_INTAKE_ANGLE, false),
                    new AdaptiveWrist(m_Intake, this::getWristAngle, true))));

    // Set default command for wrist
    m_Wrist.setDefaultCommand(new SimpleMoveWrist(m_Wrist, () -> operatorController.getLeftX()));
  }

  /** Configure POV (D-pad) buttons for operator */
  private void configurePOVButtons() {
    // Coral intake sequence (Down button)
    operatorController.povDown().onTrue(elevatorDownAndRunCoralIntake());

    // L2 scoring position (Left button)
    operatorController
        .povLeft()
        .onTrue(createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));

    // L3 scoring position (Right button)
    operatorController
        .povRight()
        .onTrue(createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));

    // L4 scoring position (Up button)
    operatorController.povUp().onTrue(createScoringSequence(Elevator.POSITION_L4, Wrist.L4_ANGLE));
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

    // Log current requested position
    Logger.recordOutput("Targets/ElevatorPosition", targetElevatorPosition);
    Logger.recordOutput("Targets/WristAngle", targetWristAngle);
  }
}
