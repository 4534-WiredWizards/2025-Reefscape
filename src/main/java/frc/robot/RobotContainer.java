// Help us Max is a  tyrant

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
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
import frc.robot.commands.Wrist.RunCoralOutake;
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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
        private final ScoringQueueSubsystem m_scoringQueue;

        // Controllers
        private final CommandXboxController operatorController = new CommandXboxController(0);
        private final Joystick driverJoystick = new Joystick(1);

        // Requested Postion
        private double targetElevatorPosition = Elevator.POSITION_L1;
        private double targetWristAngle = Wrist.L1_ANGLE;

        // Setter method for updating the target positions
        public void setTargetPositions(double elevatorPosition, double wristAngle) {
                this.targetElevatorPosition = elevatorPosition;
                this.targetWristAngle = wristAngle;
        }

        // Getter methods that can be referenced by commands
        public double getTargetElevatorPosition() {
                return targetElevatorPosition;
        }

        public double getTargetWristAngle() {
                return targetWristAngle;
        }

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        // Scoring Position

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

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                try {
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
                                drive = new Drive(
                                                new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight));

                                vision = new Vision(
                                                drive,
                                                new VisionIOLimelight(camera1Name, drive::getRotation),
                                                new VisionIOLimelight(camera0Name, drive::getRotation));
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));
                                vision = new Vision(drive, new VisionIO() {
                                }, new VisionIO() {
                                });
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                vision = new Vision(drive, new VisionIO() {
                                }, new VisionIO() {
                                });
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

                // Configure Coral Auto Scoring Buttons
                configureCoralAutoScoringButtons();
        }

        public Command createScoringSequence(double elevatorPosition, double wristAngle) {
                return new ConditionalCommand(
                                // If coral is detected in intake (sensor is triggered)
                                new SequentialCommandGroup(
                                                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                                                new ParallelDeadlineGroup(
                                                                // Use the sensor to determine when to stop waiting
                                                                new WaitUntilCommand(() -> !m_Intake.getFirstSensor()),
                                                                new SetElevatorPosition(m_elevator, elevatorPosition,
                                                                                m_Wrist, false),
                                                                new SetWristPosition(m_Wrist, wristAngle, false))),

                                // If no coral detected, just move to position without waiting for sensor
                                new SequentialCommandGroup(
                                                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                                                new ParallelCommandGroup(
                                                                new SetElevatorPosition(m_elevator, elevatorPosition,
                                                                                m_Wrist, true),
                                                                new SetWristPosition(m_Wrist, wristAngle, true))),

                                // Condition: check if sensor detects coral
                                () -> m_Intake.getFirstSensor());
        }

        // Define commadn to ElevatorDown&RuneCoralIntake
        public Command elevatorDownAndRunCoralIntake() {
                return new SequentialCommandGroup(
                                new ConditionalCommand(
                                                // If the elevator is NOT at ground position, run the full sequence
                                                new SequentialCommandGroup(
                                                                // Step 1: Clear the elevator
                                                                new SetWristPosition(m_Wrist,
                                                                                Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                                                                // Step 2: Move elevator down, prepare wrist, and run
                                                                // intake
                                                                new ParallelDeadlineGroup(
                                                                                new RunCoralIntake(m_Intake, true),
                                                                                new SetElevatorPosition(m_elevator,
                                                                                                Elevator.POSITION_GROUND,
                                                                                                m_Wrist),
                                                                                new SequentialCommandGroup(
                                                                                                new WaitUntilCommand(
                                                                                                                () -> m_elevator.getEncoderPosition() < Elevator.ELEVATOR_DANGER_LIMIT),
                                                                                                new SetWristPosition(
                                                                                                                m_Wrist,
                                                                                                                Wrist.CORAL_INTAKE_ANGLE,
                                                                                                                false)))),
                                                // If the elevator is ALREADY at ground position, just run intake and
                                                // set wrist
                                                new ParallelDeadlineGroup(
                                                                new RunCoralIntake(m_Intake, true),
                                                                new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE,
                                                                                false)),
                                                // The condition: Check if elevator is NOT at ground position
                                                () -> !m_elevator.isAtPosition(0.3))
                // new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false)
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
                NamedCommands.registerCommand(
                                "WE-L4", createScoringSequence(Elevator.POSITION_L4, Wrist.L4_ANGLE));
        }

        /** Configure event triggers for PathPlanner */
        private void configureEventTriggers() {
                // Elevator level events
                new EventTrigger("MoveToRequestedPosition")
                                .onTrue(
                                                new SequentialCommandGroup(
                                                                new SetWristPosition(m_Wrist,
                                                                                Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true),
                                                                new ParallelDeadlineGroup(
                                                                                new WaitUntilCommand(() -> !m_Intake
                                                                                                .getFirstSensor()),
                                                                                new SetElevatorPosition(
                                                                                                m_elevator,
                                                                                                () -> targetElevatorPosition, // Lambda
                                                                                                                              // expression
                                                                                                                              // that
                                                                                                                              // returns
                                                                                                                              // the
                                                                                                // current
                                                                                                // value
                                                                                                m_Wrist,
                                                                                                false),
                                                                                new SetWristPosition(m_Wrist,
                                                                                                this::getTargetWristAngle,
                                                                                                false))));
                // Commands to move wrist and elevator to the scoring position
                new EventTrigger("WE-CoralIntake").onTrue(elevatorDownAndRunCoralIntake());
                new EventTrigger("WE-L1")
                                .whileTrue(createScoringSequence(Elevator.POSITION_L1, Wrist.L1_ANGLE));
                new EventTrigger("WE-L2")
                                .whileTrue(createScoringSequence(Elevator.POSITION_L2, Wrist.L2_ANGLE));
                new EventTrigger("WE-L3")
                                .whileTrue(createScoringSequence(Elevator.POSITION_L3, Wrist.L3_ANGLE));
                new EventTrigger("WE-L4")
                                .whileTrue(createScoringSequence(Elevator.POSITION_L4, Wrist.L4_ANGLE));

                // Elevator + Wrist position events
                new EventTrigger("E-L4")
                                .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist));
                new EventTrigger("E-L4")
                                .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist));
                new EventTrigger("E-L2")
                                .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist));
                new EventTrigger("E-L4")
                                .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L1, m_Wrist));
                new EventTrigger("E-Zero")
                                .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist));
                new EventTrigger("W-L4").whileTrue(new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));
                new EventTrigger("W-L3").whileTrue(new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
                new EventTrigger("W-L2").whileTrue(new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
                new EventTrigger("W-L1").whileTrue(new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));

                // Coral intake event
                new EventTrigger("RunCoralIntake").whileTrue(new RunCoralIntake(m_Intake, true));
                new EventTrigger("RunCoralOutake")
                                .onTrue(
                                                new RunCoralOutake(m_Intake)
                                                                .withTimeout(1)
                                                                .andThen(new SetWristPosition(m_Wrist,
                                                                                Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // Basic Intake/Outake events
                new EventTrigger("AW-Outake")
                                .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false));
                new EventTrigger("AW-Intake").whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true));
        }

        /** Configure SysId characterization commands for auto chooser */
        private void configureSysIdCommands() {
                // Drive characterization commands
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
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
                // autoChooser.addOption(
                // "Wrist SysId (Dynamic Forward)",
                // m_Wrist.dynamicSysIdCommand(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Wrist SysId (Dynamic Reverse)",
                // m_Wrist.dynamicSysIdCommand(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption(
                // "Wrist SysId (Quasistatic Forward)",
                // m_Wrist.sysIdCommand(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Wrist SysId (Quasistatic Reverse)",
                // m_Wrist.sysIdCommand(SysIdRoutine.Direction.kReverse));
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

                SmartDashboard.putData("Run Coral Outake", new RunCoralOutake(m_Intake));

                // Wrist test commands
                SmartDashboard.putData("Wrist/L4", new SetWristPosition(m_Wrist, Wrist.L4_ANGLE, false));
                SmartDashboard.putData("Wrist/L3", new SetWristPosition(m_Wrist, Wrist.L3_ANGLE, false));
                SmartDashboard.putData("Wrist/L2", new SetWristPosition(m_Wrist, Wrist.L2_ANGLE, false));
                SmartDashboard.putData("Wrist/L1", new SetWristPosition(m_Wrist, Wrist.L1_ANGLE, false));
                SmartDashboard.putData("Wrist/Bottom", new SetWristPosition(m_Wrist, -203, false));
                SmartDashboard.putData(
                                "Wrist/CoralIntake", new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE));
                SmartDashboard.putData(
                                "Wrist/ClearElevator",
                                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false));
                SmartDashboard.putData(
                                "Wrist/Drive", new SetWristPosition(m_Wrist, Wrist.DRIVE_POSITION, false));

                // Elevator test commands
                SmartDashboard.putData(
                                "Elevator/Barge",
                                new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE, m_Wrist));
                SmartDashboard.putData(
                                "Elevator/L4", new SetElevatorPosition(m_elevator, Elevator.POSITION_L4, m_Wrist));
                SmartDashboard.putData(
                                "Elevator/L3", new SetElevatorPosition(m_elevator, Elevator.POSITION_L3, m_Wrist));
                SmartDashboard.putData(
                                "Elevator/L2", new SetElevatorPosition(m_elevator, Elevator.POSITION_L2, m_Wrist));
                SmartDashboard.putData(
                                "Elevator/L1", new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND, m_Wrist));

                // Drive to point test commands
                // SmartDashboard.putData(
                // "TestDrive/Zone1/Left",
                // new DriveToPoint(drive, ScoringPositions.getPose(ReefZone.ZONE_1,
                // ScoringSide.LEFT)));
                // AutoScoring test
                SmartDashboard.putData(
                                "Score/Z1-l4-L",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> vision.resetRobotPose()),
                                                new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4,
                                                                Wrist.L4_ANGLE)),
                                                new ParallelCommandGroup(
                                                                new SetWristPosition(m_Wrist,
                                                                                Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                                                                new DriveToPath(drive, Z1L)),
                                                new RunCoralOutake(m_Intake),
                                                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // Add Z1-L4 Right
                SmartDashboard.putData(

                                new SequentialCommandGroup(
                                                new InstantCommand(() -> vision.resetRobotPose()),
                                                new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4,
                                                                Wrist.L4_ANGLE)),
                                                new ParallelCommandGroup(
                                                                new SetWristPosition(m_Wrist,
                                                                                Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                                                                new DriveToPath(drive, Z1L)),
                                                new RunCoralOutake(m_Intake),
                                                new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true))

                );

                // Drive to path test commands
                SmartDashboard.putData("Drive/Z1R", new DriveToPath(drive, Z1R));
                SmartDashboard.putData("Drive/Z1L", new DriveToPath(drive, Z1L));
                SmartDashboard.putData("Drive/Z2R", new DriveToPath(drive, Z2R));
                SmartDashboard.putData("Drive/Z2L", new DriveToPath(drive, Z2L));
                SmartDashboard.putData("Drive/Z3R", new DriveToPath(drive, Z3R));
                SmartDashboard.putData("Drive/Z3L", new DriveToPath(drive, Z3L));
                SmartDashboard.putData("Drive/Z4R", new DriveToPath(drive, Z4R));
                SmartDashboard.putData("Drive/Z4L", new DriveToPath(drive, Z4L));
                SmartDashboard.putData("Drive/Z5R", new DriveToPath(drive, Z5R));
                SmartDashboard.putData("Drive/Z5L", new DriveToPath(drive, Z5L));
                SmartDashboard.putData("Drive/Z6R", new DriveToPath(drive, Z6R));
                SmartDashboard.putData("Drive/Z6L", new DriveToPath(drive, Z6L));
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
                                                                m_Wrist, m_elevator,
                                                                () -> Elevator.DOWN_DIRECTION * Elevator.MANUAL_SPEED));
                operatorController
                                .rightBumper()
                                .whileTrue(
                                                new SimpleMoveElevator(
                                                                m_Wrist, m_elevator, () -> (-1 * Elevator.DOWN_DIRECTION
                                                                                * Elevator.MANUAL_SPEED)));

                // Intake/Outtake controls
                operatorController
                                .leftTrigger()
                                .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true)); // Intake
                operatorController
                                .rightTrigger()
                                .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false)); // Outtake

                operatorController.y().whileTrue(new SimpleMoveClimb(m_climb, () -> -0.5));
                operatorController.a().whileTrue(new SimpleMoveClimb(m_climb, () -> .3));

                operatorController
                                .button(Operator.RESET_BOT_POSE_BUTTON)
                                .onTrue(new InstantCommand(() -> vision.resetRobotPose()));
                operatorController.button(Operator.RESET_BOT_POSE_BUTTON).onTrue(m_elevator.zeroCommand());

                // Configure POV buttons for operator presets
                configurePOVButtons();

                // Configure climb controls
                operatorController.y().whileTrue(new SimpleMoveClimb(m_climb, () -> -0.5));
                operatorController.a().whileTrue(new SimpleMoveClimb(m_climb, () -> .3));

                

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
        
        //Coral Auto scoring
        public void configureCoralAutoScoringButtons(){
                Trigger leftLevel1 =
                operatorController
                    .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
                    .and(operatorController.povDown());  
                Trigger leftLevel2 =
                operatorController
                    .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
                    .and(operatorController.povLeft());
                Trigger leftLevel3 =
                operatorController
                    .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
                    .and(operatorController.povRight());
                Trigger leftLevel4 =
                operatorController
                        .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
                        .and(operatorController.povUp());

                //Right Side
                Trigger rightLevel1 =
                operatorController
                    .axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
                    .and(operatorController.povDown());
                Trigger rightLevel2 =
                operatorController
                    .axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
                    .and(operatorController.povLeft());
                Trigger rightLevel3 =
                operatorController
                    .axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
                    .and(operatorController.povRight());
                Trigger rightLevel4 =
                operatorController
                        .axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
                        .and(operatorController.povUp());

                //Test Combo Buttons
                leftLevel1.onTrue(Commands.runOnce(() -> new InstantCommand(() -> System.out.println("test"))));

                //Left Side L2 - L4

                // leftLevel2.onTrue(autoScoringSequence(Constants.ScoringSide.LEFT, Constants.ScoringHeight.L2));

                // leftLevel3.onTrue(autoScoringSequence(Constants.ScoringSide.LEFT, Constants.ScoringHeight.L3));

                // leftLevel4.onTrue(autoScoringSequence(Constants.ScoringSide.LEFT, Constants.ScoringHeight.L4));

                // //Right Side L2 - L4
                // rightLevel2.onTrue(autoScoringSequence(Constants.ScoringSide.RIGHT, Constants.ScoringHeight.L2));

                // rightLevel3.onTrue(autoScoringSequence(Constants.ScoringSide.RIGHT, Constants.ScoringHeight.L3));

                // rightLevel4.onTrue(autoScoringSequence(Constants.ScoringSide.RIGHT, Constants.ScoringHeight.L4));

                
                //TODO: Delete this
                // leftLevel3.onTrue(new SequentialCommandGroup(
                //         new InstantCommand(() -> vision.resetRobotPose()),
                //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L3,
                //                         Wrist.L3_ANGLE)),
                //         new ParallelCommandGroup(
                //                         new SetWristPosition(m_Wrist,
                //                                         Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                //                         new DriveToPath(drive, Z1L)),
                //         new RunCoralOutake(m_Intake),
                //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // leftLevel4.onTrue(new SequentialCommandGroup(
                //         new InstantCommand(() -> vision.resetRobotPose()),
                //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4,
                //                         Wrist.L4_ANGLE)),
                //         new ParallelCommandGroup(
                //                         new SetWristPosition(m_Wrist,
                //                                         Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                //                         new DriveToPath(drive, Z1L)),
                //         new RunCoralOutake(m_Intake),
                //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // //Right Side L2 - L4
                // rightLevel2.onTrue(new SequentialCommandGroup(
                //         new InstantCommand(() -> vision.resetRobotPose()),
                //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L2,
                //                         Wrist.L2_ANGLE)),
                //         new ParallelCommandGroup(
                //                         new SetWristPosition(m_Wrist,
                //                                         Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                //                         new DriveToPath(drive, Z1R)),
                //         new RunCoralOutake(m_Intake),
                //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // rightLevel3.onTrue(new SequentialCommandGroup(
                //         new InstantCommand(() -> vision.resetRobotPose()),
                //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L3,
                //                         Wrist.L3_ANGLE)),
                //         new ParallelCommandGroup(
                //                         new SetWristPosition(m_Wrist,
                //                                         Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                //                         new DriveToPath(drive, Z1R)),
                //         new RunCoralOutake(m_Intake),
                //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

                // rightLevel4.onTrue(new SequentialCommandGroup(
                //         new InstantCommand(() -> vision.resetRobotPose()),
                //         new InstantCommand(() -> setTargetPositions(Elevator.POSITION_L4,
                //                         Wrist.L4_ANGLE)),
                //         new ParallelCommandGroup(
                //                         new SetWristPosition(m_Wrist,
                //                                         Wrist.MIN_CLEAR_ELEVATOR_ANGLE, false),
                //                         new DriveToPath(drive, Z1R)),
                //         new RunCoralOutake(m_Intake),
                //         new SetWristPosition(m_Wrist, Wrist.MIN_CLEAR_ELEVATOR_ANGLE, true)));

        }

        /** Configure thumb axis triggers for side-specific actions */
        // private void configureThumbAxisTriggers() {
        //         // Left side Level 1 with thumb axis left + D-pad down
        //         Trigger leftLevel1 = operatorController
        //                         .axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
        //                         .and(operatorController.povDown());

        //         leftLevel1.onTrue(Commands.runOnce(() -> new InstantCommand(() -> System.out.println("test"))));
        //         // Test command to print "test" to console

        // }

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
