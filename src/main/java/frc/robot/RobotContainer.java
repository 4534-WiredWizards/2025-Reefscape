// Copyright 2021-2025 FRC 6328
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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.C_Elevator;
import frc.robot.Constants.C_Wrist;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Wrist.AdaptiveWrist;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringQueueSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final WristSubsystem m_Wrist = new WristSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ScoringQueueSubsystem scoringQueue = new ScoringQueueSubsystem();


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Named Command

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register named commands
    NamedCommands.registerCommand("Intake", new AdaptiveWrist(m_Wrist, true));
    NamedCommands.registerCommand("SetWristPosition", new SetWristPosition(m_Wrist, 20.0));
    NamedCommands.registerCommand("Outake", new AdaptiveWrist(m_Wrist, false));

    // Event Triggers
    new EventTrigger("Elevator L4").whileTrue(new SetElevatorPosition(m_elevator, C_Elevator.toL4));
    new EventTrigger("Elevator L3").whileTrue(new SetElevatorPosition(m_elevator, C_Elevator.toL3));
    new EventTrigger("Elevator L2").whileTrue(new SetElevatorPosition(m_elevator, C_Elevator.toL2));
    new EventTrigger("Elevator L1").whileTrue(new SetElevatorPosition(m_elevator, C_Elevator.toL1));

    new EventTrigger("Wrist Coral L4").whileTrue(new SetWristPosition(m_Wrist, C_Wrist.L4));
    new EventTrigger("Wrist Coral L3").whileTrue(new SetWristPosition(m_Wrist, C_Wrist.L3));
    new EventTrigger("Wrist Coral L2").whileTrue(new SetWristPosition(m_Wrist, C_Wrist.L2));
    new EventTrigger("Wrist Coral L1").whileTrue(new SetWristPosition(m_Wrist, C_Wrist.L1));

    new EventTrigger("Outake").whileTrue(new AdaptiveWrist(m_Wrist, false));
    new EventTrigger("Intake").whileTrue(new AdaptiveWrist(m_Wrist, true));

    switch (Constants.currentMode) {
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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

            
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));


                // Max Testing 
                // Add the command to the queue
                //.onTrue(
                //         Commands.runOnce(() -> 
                //         scoringQueue.addScoringCommand(
                //             ScoringQueueSubsystem.ScoringSide.LEFT, 
                //             ScoringQueueSubsystem.ScoringHeight.L3
                //         )
                //     )
                // );

                // Clear and run the queue
                //         .onTrue(new ProcessQueueCommand(scoringQueue));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
