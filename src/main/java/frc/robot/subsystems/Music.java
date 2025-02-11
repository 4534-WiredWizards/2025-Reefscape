// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

  // Define Motors
  private final TalonFX FrontLeftDrive = new TalonFX(10);
  private final TalonFX FrontLeftTurn = new TalonFX(15);

  private final TalonFX FrontRightDrive = new TalonFX(20);
  private final TalonFX FrontRightTurn = new TalonFX(25);

  private final TalonFX BackLeftDrive = new TalonFX(30);
  private final TalonFX BackLeftTurn = new TalonFX(35);

  private final TalonFX BackRightDrive = new TalonFX(40);
  private final TalonFX BackRightTurn = new TalonFX(45);

  private Orchestra m_orchestra = new Orchestra();
  /** Creates a new Music. */
  public Music() {
    m_orchestra.addInstrument(FrontLeftDrive);
    m_orchestra.addInstrument(FrontLeftTurn);
    m_orchestra.addInstrument(FrontRightDrive);
    m_orchestra.addInstrument(FrontRightTurn);
    m_orchestra.addInstrument(BackLeftDrive);
    m_orchestra.addInstrument(BackLeftTurn);
    m_orchestra.addInstrument(BackRightDrive);
    m_orchestra.addInstrument(BackRightTurn);
  }

  public void playUnderTheSea() {
    // stop any music that is currently playing
    m_orchestra.stop();

    // load Under The Sea
    m_orchestra.loadMusic("UnderTheSea.chrp");

    // play Under The Sea
    m_orchestra.play();
  }

  public void playFinalCountdown() {
    // stop any music that is currently playing
    m_orchestra.stop();

    // load Final Countdown
    m_orchestra.loadMusic("TheFinalCountdown.chrp");

    // play Final Countdown
    m_orchestra.play();
  }

  public void playWeAreTheChampions() {
    // stop any music that is currently playing
    m_orchestra.stop();

    // load We Are The Champions
    m_orchestra.loadMusic("WeAreTheChampions.chrp");

    // play We Are The Champions
    m_orchestra.play();
  }

  public void stop() {
    m_orchestra.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
