// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.*;

// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.led.*;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;

// import frc.robot.Constants.LEDConstants;

// // import pheonix LED

// public class LED extends SubsystemBase {

// public static final CANdle Candle = new CANdle(
//   LEDConstants.LED_ID, "rio");

//   //Colors
//   public static final Color green = new Color(0,255,0);
//   public static final Color red = new Color(255,0,0);
//   public static final Color blue = new Color(0,0,255);

//   public static class Color {
//     public int red;
//     public int green;
//     public int blue;

//     public Color(int red, int green, int blue){
//       this.red = red;
//       this.green = green;
//       this.blue = blue;
//     }
//   }

//   public final int ignoreCANdleLEDS = 8;

//   public LED() {
//     CANdleConfiguration config = new CANdleConfiguration();
//     config.statusLedOffWhenActive = false;
//     config.disableWhenLOS = false;
//     config.stripType = LEDStripType.RGB;
//     config.brightnessScalar = LEDConstants.brightnessScalar;
//     config.vBatOutputMode = VBatOutputMode.Modulated;
//     Candle.configAllSettings(config, LEDConstants.timeoutMs);

//   }
// public void robotInit() {
//     LEDSegment.CandleLEDs.fullClear();
//   }

//   // Disable Robot

//   public void clearSegmentCommand(LEDSegment segment) {
//     segment.disableLEDs();
//   }

//   public static enum LEDSegment {

//     public final int startIndex;
//     public final int segmentSize;
//     public final int animationSlot;

//     private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
//       this.startIndex = startIndex;
//       this.segmentSize = segmentSize;
//       this.animationSlot = animationSlot;
//     }

//     public void fullClear() {

//       disableLEDs();
//     }

//     public void disableLEDs() {
//       Candle.setLEDs(0,0,0);
//     }

//     public void setColor(Color color) {
//       Candle.setLEDs(
//         color.red,
//         color.green,
//         color.blue,
//         100,
//         startIndex,
//         segmentSize
//       );
//     }
//   }
