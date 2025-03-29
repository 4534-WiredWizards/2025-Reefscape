// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  // Common colors
  public static class Color {
    public final int red;
    public final int green;
    public final int blue;
    public final int white;

    public Color(int red, int green, int blue) {
      this(red, green, blue, 0);
    }

    public Color(int red, int green, int blue, int white) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.white = white;
    }
  }

  // Preset colors
  public static final Color OFF = new Color(0, 0, 0);
  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color RED = new Color(255, 0, 0);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color PURPLE = new Color(128, 0, 128);
  public static final Color ORANGE = new Color(255, 165, 0);
  public static final Color CORAL_COLOR = new Color(255, 255, 255); // White for coral as requested
  public static final Color ALGAE_COLOR =
      new Color(82, 180, 157); // RGB(82, 180, 157) for algae as requested
  public static final Color TEAM_COLOR =
      new Color(229, 52, 18); // RGB(229, 52, 18) team color as requested

  // LED segment definitions
  public enum LEDSegment {
    CANDLE_LEDS(0, LED.CANDLE_LED_SEGMENT_SIZE, 0),
    MAIN_STRIP(LED.CANDLE_LED_SEGMENT_SIZE, 256, 1);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(OFF);
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, color.white, startIndex, segmentSize);
    }

    public void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }
  }

  // Static instance for usage inside LEDSegment methods
  private static CANdle candle;

  /** Creates a new LED subsystem. */
  public LEDSubsystem() {
    LEDSubsystem.candle = new CANdle(LED.LED_ID, "rio");

    // Configure the CANdle
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = LED.BRIGHTNESS_SCALAR;
    config.disableWhenLOS = false;
    config.statusLedOffWhenActive = true;
    config.vBatOutputMode = VBatOutputMode.Modulated;

    candle.configAllSettings(config, LED.TIMEOUT_MS);

    // Initialize all LEDs to off
    LEDSegment.CANDLE_LEDS.fullClear();
    LEDSegment.MAIN_STRIP.fullClear();

    // Set default state based on alliance color
    setAllianceColor();

    Logger.recordOutput("LED/Initialized", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Sets a color based on the current alliance. */
  public void setAllianceColor() {
    // Get the current alliance
    var alliance = DriverStation.getAlliance();
    Color color = alliance.isPresent() && alliance.get() == Alliance.Red ? RED : BLUE;

    // Set the color on the main strip
    LEDSegment.MAIN_STRIP.setColor(color);
    Logger.recordOutput(
        "LED/Alliance", alliance.isPresent() ? alliance.get().toString() : "Unknown");
  }

  // Robot startup fade in and out orange
  public void startupAnimation() {
    setFade(LEDSegment.MAIN_STRIP, ORANGE, 0.2);
    Logger.recordOutput("LED/Status", "Startup Animation - Orange Fade");
  }

  /** Shows team color. */
  public void showTeamColor() {
    LEDSegment.MAIN_STRIP.setColor(TEAM_COLOR);
    Logger.recordOutput("LED/Status", "Team Color");
  }

  /** Disabled mode - Rainbow animation. */
  public void disabledMode() {
    setRainbow(LEDSegment.MAIN_STRIP, 0.3);
    Logger.recordOutput("LED/Status", "Disabled Mode - Rainbow");
  }

  /** Autonomous initialization - slow fading blue. */
  public void autonomousInit() {
    setFade(LEDSegment.MAIN_STRIP, BLUE, 0.7);
    Logger.recordOutput("LED/Status", "Autonomous Init - Blue Fade");
  }

  /** Indicates elevator is moving. */
  public void elevatorMoving() {
    setStrobe(LEDSegment.MAIN_STRIP, YELLOW, 0.3);
    Logger.recordOutput("LED/Status", "Elevator Moving - Yellow Strobe");
  }

  /** Indicates setpoint reached or piece pickup/placement. */
  public void setpointReached() {
    // Solid green for setpoint reached
    LEDSegment.MAIN_STRIP.setColor(GREEN);

    // Schedule reset to idle after 1 second
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                idle();
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();

    Logger.recordOutput("LED/Status", "Setpoint Reached - Solid Green");
  }

  /** Turns off all LEDs. */
  public void off() {
    LEDSegment.CANDLE_LEDS.fullClear();
    LEDSegment.MAIN_STRIP.fullClear();
    Logger.recordOutput("LED/Status", "Off");
  }

  /** Clears all animations and sets default idle state to team color. */
  public void idle() {
    LEDSegment.MAIN_STRIP.clearAnimation();
    // LEDSegment.MAIN_STRIP.setColor(TEAM_COLOR);
    // Idle should show alliance color
    setAllianceColor();
    Logger.recordOutput("LED/Status", "Idle - Team Color");
  }

  /** Sets animation for coral intake - strobe white. */
  public void coralIntakeMode(boolean active) {
    if (active) {
      setStrobe(LEDSegment.MAIN_STRIP, CORAL_COLOR, 0.3);
      Logger.recordOutput("LED/Status", "Coral Intake - White Strobe");
    } else {
      idle();
    }
  }

  /** Sets animation for algae intake. */
  public void algaeIntakeMode(boolean active) {
    if (active) {
      setColorFlow(LEDSegment.MAIN_STRIP, ALGAE_COLOR, 0.5);
      Logger.recordOutput("LED/Status", "Algae Intake - Teal Color Flow");
    } else {
      idle();
    }
  }

  // LEDSubsystem.java

  /** Shows auto-alignment active - yellow color flow */
  public void autoAlignMode(boolean active) {
    if (active) {
      setColorFlow(LEDSegment.MAIN_STRIP, YELLOW, 0.5);
      Logger.recordOutput("LED/Status", "Auto-Align Active - Yellow Color Flow");
    } else {
      idle();
    }
  }

  /** Indicates successful intake - solid green for 1.5s. */
  public void showIntakeSuccess() {
    // Solid green for success
    LEDSegment.MAIN_STRIP.setColor(GREEN);

    // Schedule reset to idle after 1.5 seconds
    new Thread(
            () -> {
              try {
                Thread.sleep(1500);
                idle();
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();

    Logger.recordOutput("LED/Status", "Intake Success - Solid Green");
  }

  // Quick error indication - red strobe then idle
  public void showError() {
    setStrobe(LEDSegment.MAIN_STRIP, RED, 0.3);

    // Schedule reset to idle after 1 second
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                idle();
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();

    Logger.recordOutput("LED/Status", "Error - Red Strobe");
  }

  /** Shows climbing mode active - orange. */
  public void climbingMode(boolean active) {
    if (active) {
      LEDSegment.MAIN_STRIP.setColor(ORANGE);
      Logger.recordOutput("LED/Status", "Climbing Mode - Solid Orange");
    } else {
      idle();
    }
  }

  /** Shows climb finished - solid green. */
  public void climbFinished() {
    LEDSegment.MAIN_STRIP.setColor(GREEN);
    Logger.recordOutput("LED/Status", "Climb Finished - Solid Green");
  }

  /** Shows reef zone tracking active. */
  public void reefZoneTracking(boolean active) {
    if (active) {
      setRainbow(LEDSegment.MAIN_STRIP, 0.3);
      Logger.recordOutput("LED/Status", "Reef Zone Tracking - Rainbow");
    } else {
      idle();
    }
  }

  /** Sets a color flow animation. */
  public void setColorFlow(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new ColorFlowAnimation(
            color.red,
            color.green,
            color.blue,
            0,
            speed,
            segment.segmentSize,
            Direction.Forward,
            segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "ColorFlow");
  }

  /** Sets a fade animation. */
  public void setFade(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new SingleFadeAnimation(
            color.red, color.green, color.blue, 0, speed, segment.segmentSize, segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Fade");
  }

  /** Sets an RGB fade animation. */
  public void setRgbFade(LEDSegment segment, double speed) {
    segment.setAnimation(new RgbFadeAnimation(speed, segment.segmentSize, segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "RgbFade");
  }

  /** Sets a larson animation (like a scanner). */
  public void setLarson(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new LarsonAnimation(
            color.red,
            color.green,
            color.blue,
            0,
            speed,
            segment.segmentSize,
            BounceMode.Front,
            3,
            segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Larson");
  }

  /** Sets a strobe animation. */
  public void setStrobe(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new StrobeAnimation(
            color.red, color.green, color.blue, 0, speed, segment.segmentSize, segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Strobe");
  }

  /** Sets a rainbow animation. */
  public void setRainbow(LEDSegment segment, double speed) {
    segment.setAnimation(
        new RainbowAnimation(1, speed, segment.segmentSize, false, segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Rainbow");
  }

  /** Sets a twinkle animation. */
  public void setTwinkle(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new TwinkleAnimation(
            color.red,
            color.green,
            color.blue,
            0,
            speed,
            segment.segmentSize,
            TwinklePercent.Percent42));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Twinkle");
  }

  /** Sets a twinkle-off animation. */
  public void setTwinkleOff(LEDSegment segment, Color color, double speed) {
    segment.setAnimation(
        new TwinkleOffAnimation(
            color.red,
            color.green,
            color.blue,
            0,
            speed,
            segment.segmentSize,
            TwinkleOffPercent.Percent42));

    Logger.recordOutput("LED/Animation/" + segment.name(), "TwinkleOff");
  }

  /** Sets a fire animation. */
  public void setFire(LEDSegment segment, double brightness, double speed) {
    segment.setAnimation(
        new FireAnimation(brightness, speed, segment.segmentSize, 1, segment.startIndex));

    Logger.recordOutput("LED/Animation/" + segment.name(), "Fire");
  }

  /**
   * Shows different animations based on reef zone.
   *
   * @param zoneNumber Zone number (1-6)
   */
  public void showZoneIndicator(int zoneNumber) {
    // Clear any existing animations
    LEDSegment.MAIN_STRIP.clearAnimation();

    // Choose color based on zone
    Color zoneColor;
    switch (zoneNumber) {
      case 1:
        zoneColor = RED;
        break;
      case 2:
        zoneColor = GREEN;
        break;
      case 3:
        zoneColor = BLUE;
        break;
      case 4:
        zoneColor = YELLOW;
        break;
      case 5:
        zoneColor = PURPLE;
        break;
      case 6:
        zoneColor = ORANGE;
        break;
      default:
        zoneColor = WHITE;
    }

    // Set pulsing animation for the zone
    setFade(LEDSegment.MAIN_STRIP, zoneColor, 0.5);

    Logger.recordOutput("LED/Status", "Zone " + zoneNumber);
  }
}
