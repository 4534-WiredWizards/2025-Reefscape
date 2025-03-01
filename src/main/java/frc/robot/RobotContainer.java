// Copyright 2021-2025 FRC 4534
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.IO.Driver;
import frc.robot.Constants.IO.Operator;
import frc.robot.Constants.Wrist;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.SimpleMoveElevator;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.Wrist.SimpleMoveWrist;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringQueueSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  public final VisionSubsystem m_vision;
  private final WristSubsystem m_Wrist = new WristSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ScoringQueueSubsystem m_scoringQueue;

  // Controller
  private final CommandXboxController Operatorcontroller = new CommandXboxController(0);
  private final Joystick driverJoystick = new Joystick(1);

  // test 2nd xboc controller
  //   private final CommandXboxController TestController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public double getWristAngle() {
    return m_Wrist.getAngle();
  }

  // Named Command0
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register named commands
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Intake, this::getWristAngle, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand(
        "Outake", new AdaptiveWrist(m_Intake, this::getWristAngle, false));

    // Event Triggers
    new EventTrigger("Elevator L4")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L4));
    new EventTrigger("Elevator L3")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L3));
    new EventTrigger("Elevator L2")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_L2));
    new EventTrigger("Elevator L1")
        .whileTrue(new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND));

    new EventTrigger("Wrist Coral L4").whileTrue(new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));
    new EventTrigger("Wrist Coral L3").whileTrue(new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
    new EventTrigger("Wrist Coral L2").whileTrue(new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
    new EventTrigger("Wrist Coral L1").whileTrue(new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));

    new EventTrigger("Outake").whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false));
    new EventTrigger("Intake").whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true));

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
        break;
    }
    m_vision = new VisionSubsystem(drive);
    m_scoringQueue = new ScoringQueueSubsystem(drive, m_elevator, m_Wrist, m_Intake);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // TODO: Comment out after characterization
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

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(
        "TestWristCommand/Wrist L4", new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L3", new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L2", new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
    SmartDashboard.putData(
        "TestWristCommand/Wrist L1", new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));
    SmartDashboard.putData("TestWristCommand/Bottom", new SetWristPosition(m_Wrist, -203));

    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L4",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L4));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L3",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L3));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L2",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_L2));
    SmartDashboard.putData(
        "TestElevatorCommand/Elevator L1",
        new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND));

    // SmartDashboard.putData("Elevator L4", new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));
    // SmartDashboard.putData("Elevator L3", new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
    // SmartDashboard.putData("Elevator L2", new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
    // SmartDashboard.putData("Elevator L1", new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses null ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_Y_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_X_AXIS),
            () -> -driverJoystick.getRawAxis(Driver.DRIVE_ROTATE_AXIS),
            () -> driverJoystick.getRawAxis(Driver.DRIVE_THROTTLE_AXIS),
            () -> driverJoystick.getRawButton(Driver.SLOW_MODE_TOGGLE),
            () -> driverJoystick.getRawButton(Driver.FIELD_RELATIVE_TOGGLE)));

    // Lock to 0° when A button is held TODO
    // lockAngle
    new JoystickButton(driverJoystick, Driver.LOCK_ANGLE_BUTTON)
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverJoystick.getY(),
                () -> -driverJoystick.getX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // TODO make this button 8 on flight controller
    new JoystickButton(driverJoystick, Driver.STOP_WITH_X_BUTTON)
        .toggleOnTrue(Commands.runOnce(drive::stopWithX, drive));

    // Zero gyro when Y button is pressed
    new JoystickButton(driverJoystick, Driver.ZERO_GYRO_BUTTON)
        .onTrue(Commands.runOnce(() -> drive.resetGyro()).ignoringDisable(true));

    Operatorcontroller.leftBumper()
        .whileTrue(
            new SimpleMoveElevator(
                m_Wrist, m_elevator, () -> Elevator.DOWN_DIRECTION * Elevator.MANUAL_SPEED));
    Operatorcontroller.rightBumper()
        .whileTrue(
            new SimpleMoveElevator(
                m_Wrist, m_elevator, () -> (-1 * Elevator.DOWN_DIRECTION * Elevator.MANUAL_SPEED)));

    Operatorcontroller.leftTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, true)); // Outtake
    Operatorcontroller.rightTrigger()
        .whileTrue(new AdaptiveWrist(m_Intake, this::getWristAngle, false)); // pickup

    // testcontroller button
    // TestController.a().onTrue(new SetWristPosition(m_Wrist, Wrist.L1_ANGLE));
    // TestController.b().onTrue(new SetWristPosition(m_Wrist, Wrist.L2_ANGLE));
    // TestController.x().onTrue(new SetWristPosition(m_Wrist, Wrist.L3_ANGLE));
    // TestController.y().onTrue(new SetWristPosition(m_Wrist, Wrist.L4_ANGLE));

    // TestController.povDown().onTrue(new SetElevatorPosition(m_elevator, Elevator.L1_POS));
    // TestController.povRight().onTrue(new SetElevatorPosition(m_elevator, Elevator.L2_POS));
    // TestController.povLeft().onTrue(new SetElevatorPosition(m_elevator, Elevator.L3_POS));
    // TestController.povUp().onTrue(new SetElevatorPosition(m_elevator, Elevator.L4_POS));

    Constants.ScoringPositions.ZonePosition position =
        Constants.ScoringPositions.getZonePosition(
            Constants.ReefZone.ZONE_3, Constants.ScoringSide.LEFT);
    Pose2d targetPose = new Pose2d(position.x(), position.y(), new Rotation2d(position.theta()));

    // Operator on true button id SCORE_L1_BUTTON
    Operatorcontroller.button(Operator.SCORE_L1_BUTTON) // X Button
        .onTrue(new DriveToPoint(drive, targetPose));

    // Operator PID control for Nuetral D-pad
    // Operatorcontroller.povDown()
    //     .onTrue(
    //         new ParallelDeadlineGroup(
    //             new RunCoralIntake(m_Intake, true),
    //             new SetWristPosition(m_Wrist, Wrist.SAFE_WRIST_POSITION)
    //             // new SequentialCommandGroup(
    //             //     new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND),
    //             //     new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE))
    //             ));
    // Operatorcontroller.povDown()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND),
    //             new SetWristPosition(m_Wrist, Wrist.CORAL_INTAKE_ANGLE)));
    // Operatorcontroller.povUp()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new SetElevatorPosition(m_elevator, Elevator.POSITION_BARGE),
    //             new SetWristPosition(m_Wrist, Wrist.BARGE_POSITION)));
    // Operatorcontroller.povRight()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new SetElevatorPosition(m_elevator, Elevator.POSITION_GROUND),
    //             new SetWristPosition(m_Wrist, Wrist.PROCESSOR_POSITION)));

    // Buttom/Axis Event Combos
    // Right Coral Side (using RIGHT_THUMB_AXIS)
    // Trigger rightLevel1 =
    //     Operatorcontroller.axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
    //         .and(Operatorcontroller.povDown());
    // Trigger rightLevel2 =
    //     Operatorcontroller.axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
    //         .and(Operatorcontroller.povLeft());
    // Trigger rightLevel3 =
    //     Operatorcontroller.axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
    //         .and(Operatorcontroller.povRight());
    // Trigger rightLevel4 =
    //     Operatorcontroller.axisGreaterThan(Operator.RIGHT_THUMB_AXIS, 0.3)
    //         .and(Operatorcontroller.povUp());

    // // Now same for left side - also uses the right thumb axis
    // Trigger leftLevel1 =
    //     Operatorcontroller.axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
    //         .and(Operatorcontroller.povDown());
    // Trigger leftLevel2 =
    //     Operatorcontroller.axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
    //         .and(Operatorcontroller.povLeft());
    // Trigger leftLevel3 =
    //     Operatorcontroller.axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
    //         .and(Operatorcontroller.povRight());
    // Trigger leftLevel4 =
    //     Operatorcontroller.axisLessThan(Operator.RIGHT_THUMB_AXIS, -0.3)
    //         .and(Operatorcontroller.povUp());

    // // Bind the combined trigger to a command
    // // Print test log to see if the triggers are working
    // rightLevel1.onTrue(Commands.runOnce(() -> System.out.println("Right Level 1"), drive));
    // rightLevel2.onTrue(Commands.runOnce(() -> System.out.println("Right Level 2"), drive));
    // rightLevel3.onTrue(Commands.runOnce(() -> System.out.println("Right Level 3"), drive));
    // rightLevel4.onTrue(Commands.runOnce(() -> System.out.println("Right Level 4"), drive));

    // leftLevel1.onTrue(Commands.runOnce(() -> System.out.println("Left Level 1"), drive));
    // leftLevel2.onTrue(Commands.runOnce(() -> System.out.println("Left Level 2"), drive));
    // leftLevel3.onTrue(Commands.runOnce(() -> System.out.println("Left Level 3"), drive));
    // leftLevel4.onTrue(Commands.runOnce(() -> System.out.println("Left Level 4"), drive));
    // TODO: Change to

    // FIXME: Possible issues due to axis being 0-1 value, setpoint will be 20-300
    // degree postions
    m_Wrist.setDefaultCommand(new SimpleMoveWrist(m_Wrist, () -> Operatorcontroller.getLeftX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateDashboard() {
    SmartDashboard.getNumber("Match Time", DriverStation.getMatchTime());

    // Display Command buttons for Wrist and Elevator positions

  }
}
