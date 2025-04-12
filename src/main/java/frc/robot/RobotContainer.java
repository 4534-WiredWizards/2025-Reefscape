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
import frc.robot.subsystems.Music;
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

public class RobotContainer {
  private final Drive drive;
  public final Vision vision;
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final WristSubsystem m_Wrist = new WristSubsystem(m_elevator);
  public static final LEDSubsystem LEDSubsystem = new LEDSubsystem();
  public static Boolean autoDriving = false;
  private final TunerConstants.TunerSwerveDrivetrain m_swerveDrive =
      new TunerConstants.TunerSwerveDrivetrain(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  // Passes the existing motor definitions into the music subsystem
  private final Music m_music =
      new Music(
          m_swerveDrive.getModule(0).getDriveMotor(),
          m_swerveDrive.getModule(0).getSteerMotor(),
          m_swerveDrive.getModule(1).getDriveMotor(),
          m_swerveDrive.getModule(1).getSteerMotor(),
          m_swerveDrive.getModule(2).getDriveMotor(),
          m_swerveDrive.getModule(2).getSteerMotor(),
          m_swerveDrive.getModule(3).getDriveMotor(),
          m_swerveDrive.getModule(3).getSteerMotor());

  private final CommandXboxController operatorController = new CommandXboxController(0);
  private final Joystick driverThrottle = new Joystick(1);
  private final Joystick driverJoystick = new Joystick(2);

  // private final Joystick driverJoystick = new Joystick(1);

  private double targetElevatorPosition = Elevator.POSITION_L1;
  private double targetWristAngle = Wrist.L1_ANGLE;

  private final LoggedDashboardChooser<Command> autoChooser;

  public PathPlannerPath Z1R, Z1L, Z2R, Z2L, Z3R, Z3L, Z4R, Z4L, Z5R, Z5L, Z6R, Z6L;
  public PathPlannerPath Z1M, Z2M, Z3M, Z4M, Z5M, Z6M;

  public RobotContainer() {
    loadPaths();
    switch (Constants.CURRENT_MODE) {
      case REAL:
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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
    }

    registerNamedCommands();
    configureEventTriggers();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureDefaultCommands();
    configureButtonBindings();
    configureSmartDashboard();
    setupManualPoseSetter();
  }

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
      Z1R = Z1L = Z2R = Z2L = Z3R = Z3L = Z4R = Z4L = Z5R = Z5L = Z6R = Z6L = null;
      Z1M = Z2M = Z3M = Z4M = Z5M = Z6M = null;
    }
  }

  public void setTargetPositions(double elevatorPosition, double wristAngle) {
    this.targetElevatorPosition = elevatorPosition;
    this.targetWristAngle = wristAngle;
  }

  public double getTargetElevatorPosition() {
    return targetElevatorPosition;
  }

  public double getTargetWristAngle() {
    return targetWristAngle;
  }

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

  public Command driveToReefSide(ScoringSide side, BooleanSupplier cancelDriveTrigger) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              if (isAutoDriving()) {
                Logger.recordOutput(
                    "DriveToReef/Status", "Command request ignored - auto-driving active");
                System.out.println("Command request ignored - auto-driving active");
                return;
              }

              vision.resetRobotPose();
              ReefZone currentZone = drive.getZone();
              Logger.recordOutput("DriveToReef/ExecutionZone", currentZone.toString());
              Logger.recordOutput("DriveToReef/RequestedSide", side.toString());

              PathPlannerPath path = getPathForZoneAndSide(currentZone, side);
              Command pathCommand = new DriveToPath(drive, path, cancelDriveTrigger);
              pathCommand.schedule();
            }));
  }

  public Command createScoringSequence(double elevatorPosition, double wristAngle) {
    return new ConditionalCommand(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  System.out.println("Coral detected, running full sequence");
                  System.out.println("Requested Elevator Position: " + elevatorPosition);
                  System.out.println("Requested Wrist Angle: " + wristAngle);
                }),
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                new SetElevatorPosition(m_elevator, elevatorPosition, m_Wrist, false),
                new SetWristPosition(m_Wrist, wristAngle, false))),
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
        () -> {
          boolean sensorTriggered = m_Intake.getFirstSensor();
          System.out.println(
              "Sensor check: " + (sensorTriggered ? "Coral detected" : "No coral detected"));
          return sensorTriggered;
        });
  }

  public Command L1Scoring() {
    return new SequentialCommandGroup(
        new SetWristPosition(m_Wrist, 170, true),
        new RunCoralOutake(m_Intake, -0.17).withTimeout(2));
  }

  private double getZoneTargetAngle(ReefZone zone) {
    switch (zone) {
      case ZONE_1:
        return 0;
      case ZONE_2:
        return 60;
      case ZONE_3:
        return 120;
      case ZONE_4:
        return 180;
      case ZONE_5:
        return 240;
      case ZONE_6:
        return 300;
      default:
        return 0;
    }
  }

  private Command getAutoRotationCommand() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driverJoystick.getRawAxis(Driver.DRIVE_Y_AXIS),
        () -> -driverJoystick.getRawAxis(Driver.DRIVE_X_AXIS),
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          double baseAngle;
          String rotationSource = "";

          if (m_Intake.getSecondSensor()) {
            // Has coral - use zone-based angle
            ReefZone currentZone = drive.getZone();
            baseAngle = getZoneTargetAngle(currentZone);
            rotationSource = "Zone " + currentZone;

            // Flip angle for red alliance
            if (alliance == Alliance.Red) {
              baseAngle = (baseAngle + 180) % 360;
            }
          } else if (m_elevator.getEncoderPosition() < Elevator.POSITION_L2) {
            // No coral and elevator low - coral station angle
            baseAngle = 234; // Blue alliance station angle
            rotationSource = "Station";
            if (alliance == Alliance.Red) {
              baseAngle = 54; // Red alliance station angle
            }
          } else {
            // Default to current rotation
            baseAngle = drive.getRotation().getDegrees();
            rotationSource = "Current";
          }

          // Log the target angle and its source
          Logger.recordOutput("AutoRotation/TargetAngle", baseAngle);
          Logger.recordOutput("AutoRotation/Source", rotationSource);

          return Rotation2d.fromDegrees(baseAngle);
        });
  }

  /**
   * Immediately begin moving wrist to safe angle and drive elevator down while preparing to intake
   */
  private Command wristSafeElvDownThenIntake() {
    return new ParallelDeadlineGroup(
        new RunCoralIntake(m_Intake, true),
        new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
        new SequentialCommandGroup(
            new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false)
                .until(() -> m_elevator.getEncoderPosition() < Elevator.ELEVATOR_DANGER_LIMIT),
            new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false)));
  }

  /** Wait for wrist to reach safe angle before going down and intaking */
  private Command waitForSafeWristThenDownAndIntake() {
    return new SequentialCommandGroup(
        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
        new ParallelDeadlineGroup(
            new RunCoralIntake(m_Intake, true),
            new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist),
            new SequentialCommandGroup(
                new WaitUntilCommand(
                    () -> m_elevator.getEncoderPosition() < Elevator.ELEVATOR_DANGER_LIMIT),
                new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false))));
  }

  /** Set wrist to intake position and run intake, assuming already safe */
  private Command wristInAndIntake() {
    return new ParallelDeadlineGroup(
        new RunCoralIntake(m_Intake, true),
        new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE, false));
  }

  public Command elevatorDownAndRunCoralIntake(boolean addRumbleFeedback) {
    Command mainCommand =
        new ConditionalCommand(
            new ConditionalCommand(
                wristSafeElvDownThenIntake(),
                waitForSafeWristThenDownAndIntake(),
                () ->
                    m_Wrist.getAngle() < Wrist.MIN_CLEAR_ELEVATOR_ANGLE && m_Wrist.getAngle() > 82),
            wristInAndIntake(),
            () -> !(m_elevator.getEncoderPosition() < 3.0));

    return addRumbleFeedback ? mainCommand.andThen(setOperatorRumble(0.2)) : mainCommand;
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand(
        "Outake", new AdaptiveWrist(m_Intake, this::getWristAngle, false));
    NamedCommands.registerCommand("WE-CoralIntake", elevatorDownAndRunCoralIntake(false));
    NamedCommands.registerCommand(
        "WE-L1", createScoringSequence(Elevator.POSITION_L1, Wrist.L1_ANGLE));
    NamedCommands.registerCommand(
        "WE-L2", createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));
    NamedCommands.registerCommand(
        "WE-L3", createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));
    NamedCommands.registerCommand("WE-L4", createScoringSequence(62.9, 110.0));
    NamedCommands.registerCommand(
        "E-Zero", new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist, true));
    NamedCommands.registerCommand(
        "SetWristSafePosition",
        new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true));
    NamedCommands.registerCommand(
        "SetTargetL4",
        new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4, Wrist.L4_ANGLE)));
    NamedCommands.registerCommand("RunCoralOutake", new RunCoralOutake(m_Intake).withTimeout(1.5));
    NamedCommands.registerCommand(
        "ResetBotPose", new InstantCommand(() -> vision.resetRobotPose()));
  }

  private void configureEventTriggers() {
    new EventTrigger("MoveToRequestedPosition")
        .onTrue(
            new SequentialCommandGroup(
                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                    new SetElevatorPosition(
                        m_elevator, this::getTargetElevatorPosition, m_Wrist, false),
                    new SetWristPosition(m_Wrist, this::getTargetWristAngle, false))));

    new EventTrigger("WE-CoralIntake").onTrue(elevatorDownAndRunCoralIntake(false));
  }

  private void setupManualPoseSetter() {
    SmartDashboard.putNumber("ManualPose/X", 7.0);
    SmartDashboard.putNumber("ManualPose/Y", 2.0);
    SmartDashboard.putNumber("ManualPose/Rotation", 180);
    SmartDashboard.putData("ManualPose/SetPose", new ManualPoseSetter(drive).ignoringDisable(true));
  }

  private void configureSmartDashboard() {
    SmartDashboard.putNumber("DriveAngle/TargetDegrees", 0);
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
                  System.out.println("[Button 2] Attempting reset...");
                  vision.resetRobotPose();
                })
            .ignoringDisable(true));

    SmartDashboard.putData("LED/Off", new InstantCommand(() -> LEDSubsystem.off()));
    SmartDashboard.putData(
        "LED/AllianceColor", new InstantCommand(() -> LEDSubsystem.setAllianceColor()));
  }

  public Command setOperatorRumble(double rumble) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> operatorController.setRumble(kBothRumble, rumble)),
            new WaitCommand(0.3))
        .finallyDo(interrupted -> operatorController.setRumble(kBothRumble, 0.0));
  }

  private void configureButtonBindings() {
    Trigger cancelDriveTrigger = new JoystickButton(driverJoystick, 6);

    new JoystickButton(driverJoystick, 2)
        .onTrue(Commands.runOnce(() -> vision.resetRobotPose()).ignoringDisable(true));

    new JoystickButton(driverJoystick, 14)
        .onTrue(driveToReefSide(ScoringSide.LEFT, cancelDriveTrigger));
    new JoystickButton(driverJoystick, 12)
        .onTrue(driveToReefSide(ScoringSide.RIGHT, cancelDriveTrigger));
    new JoystickButton(driverThrottle, 4)
        .onTrue(driveToReefSide(ScoringSide.MIDDLE, cancelDriveTrigger));
    new JoystickButton(driverThrottle, 5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  double targetX =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? Constants.FieldPosition.Blue.Barge.SCORING_X
                          : Constants.FieldPosition.Red.Barge.SCORING_X;
                  double currentY = drive.getPose().getY();
                  double targetRotation =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                          ? Math.toRadians(180)
                          : Math.toRadians(0);
                  Pose2d targetPose = new Pose2d(targetX, currentY, new Rotation2d(targetRotation));
                  new DriveToPoint(drive, targetPose, cancelDriveTrigger).schedule();
                }));

    new JoystickButton(driverJoystick, Driver.RightJoystick.TRIGGER)
        .whileTrue(getAutoRotationCommand());

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

    operatorController
        .leftTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    operatorController
        .rightTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false));

    operatorController.button(Operator.PRESS_LEFT_THUMBSTICK).onTrue(L1Scoring());
    operatorController
        .button(Operator.PRESS_RIGHT_THUMBSTICK)
        .onTrue(new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE));
    operatorController.button(Operator.ZERO_ELEVATOR_BUTTON).onTrue(m_elevator.zeroCommand());

    configurePOVButtons();

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

    operatorController
        .y()
        .onTrue(
            new SequentialCommandGroup(
                new SetElevatorPosition(m_elevator, Elevator.POSITION_SAFE_ALGAE, m_Wrist, false)));

    operatorController
        .x()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE, m_Wrist, true),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(
                            () -> m_elevator.getEncoderPosition() > Elevator.POSITION_BARGE - 36),
                        new AdaptiveWrist(m_Intake, () -> Wrist.BARGE_ANGLE, false)
                            .withTimeout(.3))),
                wristSafeElvDownThenIntake()));
  }

  private void configurePOVButtons() {
    operatorController.povDown().onTrue(elevatorDownAndRunCoralIntake(true));
    operatorController
        .povLeft()
        .onTrue(createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));
    operatorController
        .povRight()
        .onTrue(createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));
    operatorController.povUp().onTrue(createScoringSequence(Elevator.POSITION_L4, Wrist.L4_ANGLE));
  }

  private void configureDefaultCommands() {
    // drive.setDefaultCommand(getAutoRotationCommand());
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_Y_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_X_AXIS),
            () -> -(driverJoystick.getRawAxis(4) + driverJoystick.getRawAxis(2)),
            () -> driverThrottle.getRawAxis(0),
            () -> driverThrottle.getRawButton(1),
            () -> false));
    m_Intake.setDefaultCommand(m_Intake.getProtectionCommand());
    m_Wrist.setDefaultCommand(new SimpleMoveWrist(m_Wrist, () -> operatorController.getLeftX()));
  }

  public Command getAutonomousCommand() {
    System.err.println("Running autonomous command!");
    return autoChooser.get();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    Pose2d currentPose = drive.getPose();
    SmartDashboard.putNumber("Pose/X", currentPose.getX());
    SmartDashboard.putNumber("Pose/Y", currentPose.getY());
    SmartDashboard.putNumber("Pose/Rotation", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Drive/CurrentZone", drive.getZone().ordinal() + 1);
    SmartDashboard.putBoolean("AutoDriving", isAutoDriving());
    Logger.recordOutput("Targets/ElevatorPosition", targetElevatorPosition);
    Logger.recordOutput("Targets/WristAngle", targetWristAngle);
  }

  public double getWristAngle() {
    return m_Wrist.getAngle();
  }

  public boolean isAutoDriving() {
    return autoDriving;
  }

  public static void setAutoDriving(boolean autoDriving) {
    RobotContainer.autoDriving = autoDriving;
  }
}
