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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPath;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.SimpleMoveElevator;
import frc.robot.commands.ManualPoseSetter;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.RunCoralIntake;
import frc.robot.commands.Wrist.RunCoralOutake;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.Wrist.SimpleMoveWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
import java.util.function.BooleanSupplier;
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
  // public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
  public static Boolean autoDriving = false;

  // Controllers
  private final CommandXboxController operatorController = new CommandXboxController(0);
  // private final CommandXboxController operatorController2 = new
  // CommandXboxController(2);

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

  // pathplanning paths for Algea (middle)
  public PathPlannerPath Z1M;
  public PathPlannerPath Z2M;
  public PathPlannerPath Z3M;
  public PathPlannerPath Z4M;
  public PathPlannerPath Z5M;
  public PathPlannerPath Z6M;

  public double getWristAngle() {
    return m_Wrist.getAngle();
  }

  public boolean isAutoDriving() {
    return autoDriving;
  }

  public static void setAutoDriving(boolean autoDriving) {

    RobotContainer.autoDriving = autoDriving;
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

    // Default commands
    configureDefaultCommands();

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

      // Load paths for algae scoring
      Z1M = PathPlannerPath.fromPathFile("1M");
      Z2M = PathPlannerPath.fromPathFile("2M");
      Z3M = PathPlannerPath.fromPathFile("3M");
      Z4M = PathPlannerPath.fromPathFile("4M");
      Z5M = PathPlannerPath.fromPathFile("5M");
      Z6M = PathPlannerPath.fromPathFile("6M");

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

      Z1M = null;
      Z2M = null;
      Z3M = null;
      Z4M = null;
      Z5M = null;
      Z6M = null;
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
  // public PathPlannerPath getPathForZoneAndSide(ReefZone zone, ScoringSide side)
  // {
  // System.out.println("\n[Path Selection] Determining path:");
  // System.out.println("-> Current Zone: " + zone);
  // System.out.println("-> Requested Side: " + side);
  // System.out.println("-> Alliance: " +
  // DriverStation.getAlliance().orElse(Alliance.Blue));

  // PathPlannerPath path =
  // switch (zone) {
  // case ZONE_1 -> side == ScoringSide.RIGHT ? Z1R : Z1L;
  // case ZONE_2 -> side == ScoringSide.RIGHT ? Z2R : Z2L;
  // case ZONE_3 -> side == ScoringSide.RIGHT ? Z3R : Z3L;
  // case ZONE_4 -> side == ScoringSide.RIGHT ? Z4R : Z4L;
  // case ZONE_5 -> side == ScoringSide.RIGHT ? Z5R : Z5L;
  // case ZONE_6 -> side == ScoringSide.RIGHT ? Z6R : Z6L;
  // default -> null;
  // };

  // if (path != null) {
  // System.out.println("-> Selected Path: " + path.name);
  // } else {
  // System.err.println("!! ERROR: No path found for zone " + zone + " side " +
  // side);
  // }

  // return path;
  // }

  public PathPlannerPath getPathForZoneAndSide(ReefZone zone, ScoringSide side) {
    System.out.println("\n[Path Selection] Determining path:");
    System.out.println("-> Current Zone: " + zone);
    System.out.println("-> Requested Side: " + side);
    System.out.println("-> Alliance: " + DriverStation.getAlliance().orElse(Alliance.Blue));

    PathPlannerPath path =
        switch (zone) {
          case ZONE_1 -> switch (side) {
            case RIGHT -> Z1R;
            case LEFT -> Z1L;
            case MIDDLE -> Z1M;
          };
          case ZONE_2 -> switch (side) {
            case RIGHT -> Z2R;
            case LEFT -> Z2L;
            case MIDDLE -> Z2M;
          };
          case ZONE_3 -> switch (side) {
            case RIGHT -> Z3R;
            case LEFT -> Z3L;
            case MIDDLE -> Z3M;
          };
          case ZONE_4 -> switch (side) {
            case RIGHT -> Z4R;
            case LEFT -> Z4L;
            case MIDDLE -> Z4M;
          };
          case ZONE_5 -> switch (side) {
            case RIGHT -> Z5R;
            case LEFT -> Z5L;
            case MIDDLE -> Z5M;
          };
          case ZONE_6 -> switch (side) {
            case RIGHT -> Z6R;
            case LEFT -> Z6L;
            case MIDDLE -> Z6M;
          };
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

  // public Command driveToReefSide2(ScoringSide side) {
  // return new SequentialCommandGroup(
  // new InstantCommand(() -> vision.resetRobotPose()),
  // Commands.runOnce(() -> {
  // Logger.recordOutput("DriveToReef/RequestedSide", side.toString());
  // ReefZone currentZone = drive.getZone();
  // Logger.recordOutput("DriveToReef/ExecutionZone", currentZone.toString());
  // }),
  // new DriveToPath(drive, getPathForZoneAndSide(drive.getZone(), side))
  // .until(new JoystickButton(driverJoystick, Driver.RightJoystick.TRIGGER)),
  // setOperatorRumble(0.7)
  // ).finallyDo(() -> {
  // drive.stop();
  // operatorController.setRumble(kBothRumble, 0.0);
  // });
  // }

  public Command driveToReefSide(ScoringSide side, BooleanSupplier cancelDriveTrigger) {
    return new SequentialCommandGroup(
        // First, check if we can run the command
        new InstantCommand(
            () -> {
              if (isAutoDriving()) {
                Logger.recordOutput(
                    "DriveToReef/Status", "Command request ignored - auto-driving active");
                System.out.println("Command request ignored - auto-driving active");
                return;
              }

              // Continue with normal execution
              vision.resetRobotPose();

              // Get the current zone at execution time
              ReefZone currentZone = drive.getZone();
              Logger.recordOutput("DriveToReef/ExecutionZone", currentZone.toString());
              Logger.recordOutput("DriveToReef/RequestedSide", side.toString());

              // Get the path for the current zone and side
              PathPlannerPath path = getPathForZoneAndSide(currentZone, side);

              // Create and schedule the command
              Command pathCommand = new DriveToPath(drive, path, cancelDriveTrigger);
              pathCommand.schedule();
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

  public Command L1Scoring() {
    return new SequentialCommandGroup(
        new SetWristPosition(m_Wrist, 165, true),
        new RunCoralOutake(m_Intake, -0.148).withTimeout(2)
        // new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, true
        );
  }

  // new SequentialCommandGroup(
  // // Step 1: Clear the elevator
  // new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
  // // Step 2: Move elevator down, prepare wrist, and run intake
  // new ParallelDeadlineGroup(
  // new RunCoralIntake(m_Intake, true),
  // new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
  // new SequentialCommandGroup(
  // new WaitUntilCommand(
  // () -> m_elevator.getEncoderPosition() <
  // Elevator.ELEVATOR_DANGER_LIMIT),
  // new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)))),

  /** Creates a command sequence for elevator down and coral intake */
  public Command elevatorDownAndRunCoralIntake(boolean addRumbleFeedback) {
    Command mainCommand =
        new ConditionalCommand(
            // If the elevator is NOT at ground position, run the full sequence
            new ConditionalCommand(
                new ParallelDeadlineGroup(
                    new RunCoralIntake(m_Intake, true),
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
                    new SequentialCommandGroup(
                        // Start with wrist in clear position
                        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false)
                            // Cancel this when elevator is below danger limit
                            .until(
                                () ->
                                    m_elevator.getEncoderPosition()
                                        < Elevator.ELEVATOR_DANGER_LIMIT),
                        // Then move wrist to intake position
                        new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false))),
                new SequentialCommandGroup( // Run when wrist is Not cleaxxr
                    // Clear the wrist
                    new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                    // Move elevator down and move wrist in and run intake
                    new ParallelDeadlineGroup(
                        new RunCoralIntake(m_Intake, true),
                        new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(
                                () ->
                                    m_elevator.getEncoderPosition()
                                        < Elevator.ELEVATOR_DANGER_LIMIT),
                            new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)))),
                () ->
                    m_Wrist.getAngle() < Wrist.MIN_CLEAR_ELEVATOR_ANGLE && m_Wrist.getAngle() > 82),
            // If the elevator is ALREADY at ground position, just run intake and set wrist
            new ParallelDeadlineGroup(
                new RunCoralIntake(m_Intake, true),
                new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)),
            // The condition: Check if elevator is NOT at ground position
            () -> !(m_elevator.getEncoderPosition() < 3.0));

    // Conditionally add rumble feedback based on the parameter
    if (addRumbleFeedback) {
      return mainCommand.andThen(setOperatorRumble(0.2));
    } else {
      return mainCommand;
    }
  }

  /** Register named commands for PathPlanner */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand(
        "Outake", new AdaptiveWrist(m_Intake, this::getWristAngle, false));

    // Complex WE-CoralIntake command for intake with elevator coordination
    NamedCommands.registerCommand("WE-CoralIntake", elevatorDownAndRunCoralIntake(false));

    // Register different level scoring commands
    NamedCommands.registerCommand(
        "WE-L1", createScoringSequence(Elevator.POSITION_L1, Wrist.L1_ANGLE));
    NamedCommands.registerCommand(
        "WE-L2", createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));
    NamedCommands.registerCommand(
        "WE-L3", createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));
    NamedCommands.registerCommand("WE-L4", createScoringSequence(62.9, 110.0));

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
        "ResetBotPose", new InstantCommand(() -> vision.resetRobotPose()));
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
    new EventTrigger("WE-CoralIntake").onTrue(elevatorDownAndRunCoralIntake(false));
  }

  /** Setup manual pose setter functionality */
  private void setupManualPoseSetter() {
    // Initialize default values for manual pose setter
    SmartDashboard.putNumber("ManualPose/X", 7.0);
    SmartDashboard.putNumber("ManualPose/Y", 2.0);
    SmartDashboard.putNumber("ManualPose/Rotation", 180);
    SmartDashboard.putData("ManualPose/SetPose", new ManualPoseSetter(drive).ignoringDisable(true));
  }

  /** Configure SmartDashboard commands and test controls */
  private void configureSmartDashboard() {

    // Test print command with ignoringDisable
    // SmartDashboard.putData(
    // "Reset Pose Test",
    // new InstantCommand(
    // () -> {
    // System.out.println("Attempting to reset pose...");
    // vision.resetRobotPose();
    // })
    // .ignoringDisable(true));

    // SmartDashboard.putData(
    // "Reset Pos Test 2",
    // new InstantCommand(() -> vision.resetRobotPose())
    // .ignoringDisable(true) // Allows execution while disabled
    // .withName("ResetVisionPose"));

    // SmartDashboard.putData("Run Coral Outake", new RunCoralOutake(m_Intake));

    // Wrist test commands
    // SmartDashboard.putData("Wrist/L4", new SetWristPosition(m_Wrist,
    // Wrist.L4_ANGLE, false));
    // SmartDashboard.putData("Wrist/L3", new SetWristPosition(m_Wrist,
    // Wrist.L3_ANGLE, false));
    // SmartDashboard.putData("Wrist/L2", new SetWristPosition(m_Wrist,
    // Wrist.L2_ANGLE, false));
    // SmartDashboard.putData("Wrist/L1", new SetWristPosition(m_Wrist,
    // Wrist.L1_ANGLE, false));
    // SmartDashboard.putData(
    // "Wrist/CoralIntake", new SetWristPosition(m_Wrist,
    // Wrist.CORAL_INTAKE_ANGLE));
    // SmartDashboard.putData(
    // "Wrist/ClearElevator",
    // new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false));
    // SmartDashboard.putData(
    // "Wrist/Drive", new SetWristPosition(m_Wrist, Wrist.DRIVE_POSITION, false));

    // Elevator test commands
    // SmartDashboard.putData(
    // "Elevator/Barge", new SetElevatorPosition(m_elevator,
    // Elevator.POSITION_BARGE, m_Wrist));
    // SmartDashboard.putData(
    // "Elevator/L4", new SetElevatorPosition(m_elevator, Elevator.POSITION_L4,
    // m_Wrist));
    // SmartDashboard.putData(
    // "Elevator/L3", new SetElevatorPosition(m_elevator, Elevator.POSITION_L3,
    // m_Wrist));
    // SmartDashboard.putData(
    // "Elevator/L2", new SetElevatorPosition(m_elevator, Elevator.POSITION_L2,
    // m_Wrist));
    // SmartDashboard.putData(
    // "Elevator/L1", new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND,
    // m_Wrist));

    SmartDashboard.putData(
        "PoseReset/1",
        new InstantCommand(
                () -> {
                  System.out.println("[Button 1] Attempting reset...");
                  vision.resetRobotPose();
                })
            .ignoringDisable(true));

    SmartDashboard.putData(
        "PoseReset/2",
        new InstantCommand(
                () -> {
                  System.out.println("[Button 2] Attempting reset..."); // Add print
                  vision.resetRobotPose();
                })
            .ignoringDisable(true));

    // Set led color commands

    SmartDashboard.putData(
        "LED/Off",
        new InstantCommand(
            () -> {
              LEDSubsystem.off();
            }));

    // setAllianceColor
    SmartDashboard.putData(
        "LED/AllianceColor",
        new InstantCommand(
            () -> {
              LEDSubsystem.setAllianceColor();
            }));

    // New method with cancellation capability
    // SmartDashboard.putData("Score/AutoZone L", driveToReefSide(ScoringSide.LEFT,
    // () -> false));
    // SmartDashboard.putData("Score/AutoZone R", driveToReefSide(ScoringSide.RIGHT,
    // () -> false));

    // Test hold climb button

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
            new WaitCommand(0.3))
        .finallyDo(interrupted -> operatorController.setRumble(kBothRumble, 0.0));
  }

  /** Configure button bindings for driver and operator controls */
  private void configureButtonBindings() {
    // Default command for drive - joystick control
    Trigger cancelDriveTrigger = new JoystickButton(driverJoystick, Driver.RightJoystick.TRIGGER);

    // Lock to 0° when lock angle button is held
    // new JoystickButton(driverJoystick, Driver.LeftThrottle.BOTTOM_THUMB_BUTTON)
    // .toggleOnTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -driverJoystick.getY(),
    // () -> -driverJoystick.getX(),
    // () -> new Rotation2d(60)));

    // Switch to X pattern when X button is pressed
    // new JoystickButton(driverJoystick, Driver.LeftThrottle.MIDDLE_THUMB_BUTTON)
    // .whileTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero gyro when reset button is pressed
    new JoystickButton(driverJoystick, Driver.RightJoystick.STRIPED_CENTER_BUTTON)
        .onTrue(Commands.runOnce(() -> vision.resetRobotPose()).ignoringDisable(true));

    // Add driver joystick commands for reef side approachs
    new JoystickButton(
            driverJoystick,
            Driver.BASE_LEFT_BUTTON) // Auto align with left reef post in current zone
        .onTrue(driveToReefSide(ScoringSide.LEFT, cancelDriveTrigger));
    new JoystickButton(
            driverJoystick,
            Driver.BASE_RIGHT_BUTTON) // Auto align with right reef post in current zone
        .onTrue(driveToReefSide(ScoringSide.RIGHT, cancelDriveTrigger));
    // new JoystickButton(
    // driverJoystick,
    // Driver.LeftThrottle
    // .MIDDLE_THUMB_BUTTON) // Auto align with left reef post in current zone
    // .onTrue(driveToReefSide(ScoringSide.LEFT, cancelDriveTrigger));
    // new JoystickButton(
    // driverJoystick,
    // Driver.LeftThrottle
    // .BOTTOM_THUMB_BUTTON) // Auto align with right reef post in current zone
    // .onTrue(driveToReefSide(ScoringSide.RIGHT, cancelDriveTrigger));

    new JoystickButton(
            driverJoystick,
            Driver.LeftThrottle.TOP_THUMB_BUTTON) // Algae pickup on reef in current zone
        .onTrue(driveToReefSide(ScoringSide.MIDDLE, cancelDriveTrigger));
    // Add this button binding in your configureButtonBindings() method
    new JoystickButton(
            driverJoystick, Driver.LeftThrottle.FRONT_THUMB_BUTTON) // Drive to barge position
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Create the target pose based on alliance
                  double targetX =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? Constants.FieldPosition.Blue.Barge.SCORING_X
                          : Constants.FieldPosition.Red.Barge.SCORING_X;

                  // Keep the current Y position to maintain lateral position
                  double currentY = drive.getPose().getY();

                  // Use 180 degrees rotation to face the barge
                  double targetRotation =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? Math.toRadians(180)
                          : Math.toRadians(0);
                  Pose2d targetPose = new Pose2d(targetX, currentY, new Rotation2d(targetRotation));

                  // Create and schedule the command
                  new DriveToPoint(drive, targetPose, cancelDriveTrigger).schedule();
                }));

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

    operatorController.button(Operator.PRESS_LEFT_THUMBSTICK).onTrue(L1Scoring());
    operatorController
        .button(Operator.PRESS_RIGHT_THUMBSTICK)
        .onTrue(new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE));

    // operatorController
    // .button(Operator.RESET_BOT_POSE_BUTTON)
    // .onTrue(new InstantCommand(() ->
    // vision.resetRobotPose()).ignoringDisable(true));
    operatorController.button(Operator.ZERO_ELEVATOR_BUTTON).onTrue(m_elevator.zeroCommand());

    // Configure POV buttons for operator presets
    configurePOVButtons();

    // Hold Climb Button
    // operatorController
    // .button(Operator.RESET_BOT_POSE_BUTTON)
    // .toggleOnTrue(new HoldClimbPosition(m_climb));

    // // Configure climb controls
    // operatorController2 // Top left stock 1
    // .a()Finished run coral intake

    // .onTrue(
    // new ParallelCommandGroup(
    // new SimpleMoveClimb(m_climb, () -> -.85),
    // new SetWristPosition(m_Wrist, Wrist.CLIMB_ANGLE, false)));
    // // .toggleOnTrue(new SimpleMoveClimb(m_climb, () -> -0.6)); // Wind - Climb
    // // until reverse limit reached and if it slips then rewind

    // operatorController2 // Bottom left stock 1
    // .b()
    // .whileTrue(new SimpleMoveClimb(m_climb, () -> 1)); // Unwind //Back stock 2

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
                    new AdaptiveWrist(m_Intake, () -> Wrist.ALGAE_INTAKE_ANGLE, true))));

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
                    new AdaptiveWrist(m_Intake, () -> Wrist.ALGAE_INTAKE_ANGLE, true))));

    // Safe storage position for algae
    operatorController
        .y()
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, (Wrist.BARGE_ANGLE - 5.0), true),
                new SetElevatorPosition(m_elevator, Elevator.POSITION_SAFE_ALGAE, m_Wrist, true)));

    // Shoot algae
    operatorController
        .x()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // Command to move elevator to BARGE position and adjust wrist
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE, m_Wrist, true),

                    // Sequence to wait for elevator position and trigger algae outtake
                    new SequentialCommandGroup(
                        // Wait until elevator is above the threshold
                        new WaitUntilCommand(
                            () -> m_elevator.getEncoderPosition() > Elevator.POSITION_BARGE - 36),

                        // Run the algae outtake command (replace with your specific command)
                        new AdaptiveWrist(m_Intake, () -> Wrist.BARGE_ANGLE, false)
                            .withTimeout(.5))),
                // Wait for 0.2 seconds
                // new WaitCommand(0.2),
                // Move elevator to zero position
                elevatorDownAndRunCoralIntake(false)));

    // Set default command for wrist
  }

  /** Configure POV (D-pad) buttons for operator */
  private void configurePOVButtons() {
    // Coral intake sequence (Down button)
    operatorController.povDown().onTrue(elevatorDownAndRunCoralIntake(true));

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

  // Function to configure default commands for subsystems
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_Y_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_X_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_ROTATE_AXIS),
            () -> driverJoystick.getRawAxis(Driver.DRIVE_THROTTLE_AXIS),
            () -> driverJoystick.getRawButton(Driver.RightJoystick.RIGHT_THUMB_BUTTON),
            () -> false));
    m_Intake.setDefaultCommand(m_Intake.getProtectionCommand());
    m_Wrist.setDefaultCommand(new SimpleMoveWrist(m_Wrist, () -> operatorController.getLeftX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.err.println("Running autonomous command!");
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

    SmartDashboard.putBoolean("AutoDriving", isAutoDriving());
    // Log current requested position
    Logger.recordOutput("Targets/ElevatorPosition", targetElevatorPosition);
    Logger.recordOutput("Targets/WristAngle", targetWristAngle);
  }
}
