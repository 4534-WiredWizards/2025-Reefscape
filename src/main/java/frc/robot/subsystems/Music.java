// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Tilghman97WasHere

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {
  private Orchestra m_orchestra = new Orchestra();

  /** Creates a new Music subsystem that uses existing motor references */
  public Music(
      TalonFX frontLeftDrive,
      TalonFX frontLeftTurn,
      TalonFX frontRightDrive,
      TalonFX frontRightTurn,
      TalonFX backLeftDrive,
      TalonFX backLeftTurn,
      TalonFX backRightDrive,
      TalonFX backRightTurn) {

    // Add motors to orchestra using existing references
    m_orchestra.addInstrument(frontLeftDrive, 0);
    m_orchestra.addInstrument(frontLeftTurn, 0);
    m_orchestra.addInstrument(frontRightDrive, 0);
    m_orchestra.addInstrument(frontRightTurn, 0);

    // Add back motors to different track to add ability for chords
    m_orchestra.addInstrument(backLeftDrive, 0);
    m_orchestra.addInstrument(backLeftTurn, 0);
    m_orchestra.addInstrument(backRightDrive, 0);
    m_orchestra.addInstrument(backRightTurn, 0);
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

  public void playNotLikeUs() {
    // stop any music that is currently playing
    m_orchestra.stop();

    // load Not Like Us
    m_orchestra.loadMusic("NotLikeUs.chrp");

    // play Not Like Us
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
