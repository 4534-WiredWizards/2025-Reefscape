// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import frc.robot.subsystems.ElevatorSubsystem;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class SimpleMoveElevator extends Command {

//   private final ElevatorSubsystem m_elevator;
//   private final DoubleSupplier speedSupplier;

//   public SimpleMoveElevator(ElevatorSubsystem m_elevator, DoubleSupplier speedSupplier) {
//     this.m_elevator = m_elevator;
//     this.speedSupplier = speedSupplier;
//     addRequirements(m_elevator);
//   }

//   @Override
//   public void initialize() {
//     // Stop the PID controller when manual control starts
//     m_elevator.stop();
//   }

//   @Override
//   public void execute() {
//     // Move the elevator manually based on the input speed

//     double speed = speedSupplier.getAsDouble();
//     m_elevator.moveManual(speed);

//   }

//   @Override
//   public void end(boolean interrupted) {
//     // Stop the elevator when the command ends

//     m_elevator.stop();
//   }

//   @Override
//   public boolean isFinished() {
//     // This command runs continuously until interrupted

//     return false;
//   }
// }
