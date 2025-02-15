package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringQueue;
import frc.robot.commands.CoralScoringCommand;
import frc.robot.subsystems.drive.Drive;


public class ScoringQueueSubsystem extends SubsystemBase {
  private final Queue<CoralScoringCommand> commandQueue = new LinkedList<>();
  private final Drive swerveDrive;

  //enstanciate Zone Detection
  private double BLUECENTERX;
  private double BLUECENTERY;

  private final double[] lineSlopes;
  

  // Random stuff for SmartDashboard
  private static final double EPSILON = 1e-6;
  private boolean Zone1Top = false;
  private boolean Zone2Top = false;
  private boolean Zone3Top = false;


  
  public ScoringQueueSubsystem(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.BLUECENTERX = ScoringQueue.BLUECENTERX;
    this.BLUECENTERY = ScoringQueue.BLUECENTERY;
    this.lineSlopes = new double[3];

    lineSlopes[0] = Double.POSITIVE_INFINITY; //vertical line
    lineSlopes[1] = Math.tan(Math.toRadians(60)); //60 degrees
    lineSlopes[2] = Math.tan(Math.toRadians(-60)); //-60 degrees 
    SmartDashboard.putNumber("CURRENT_ZONE", 0);
  }


  public enum ScoringSide {
    LEFT,
    RIGHT
  }

  public enum ReefZone {
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4,
    ZONE_5,
    ZONE_6,  
  }

  public enum ScoringHeight {
    L1,
    L2,
    L3,
    L4
  }

  public void addScoringCommand(ScoringSide side, ScoringHeight height) {
    System.out.println("Adding command: " + side + " " + height);

    Pose2d pose = swerveDrive.getPose();
    
    // Each zone is defined by a trapaziod shape with 4 corners
    // If the robots pose is within the zone, set the robots zone to that zone number
    // If the robots pose is not within any zone, set the robots zone to 0

    // Zone 1
    // Need to determine zone based on pose
    //
    // if(pose > Zone1Leftbound && pose < Zone1LightBound){

    // }else if( pose > Zone2LeftBound && pose < Zone2RightBound{
    
    // }

    
    commandQueue.add(new CoralScoringCommand(side, height));
  }




  public Queue<CoralScoringCommand> getQueue() {
    return new LinkedList<>(commandQueue);
  }

  public void clearQueue() {
    commandQueue.clear();
  }

  @Override
  public void periodic() {
    // Update level status indicators
    updateLevelStatus();
    zoneTest();

  }

  private  void zoneTest () {
    Pose2d pose = swerveDrive.getPose();
    ReefZone zone = getZone(pose);
    switch (zone) {
      case ZONE_1 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 1);
      }
      case ZONE_2 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 2);
      }
      case ZONE_3 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 3);
      }
      case ZONE_4 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 4);
      }
      case ZONE_5 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 5);
      }
      case ZONE_6 -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 6);
      }
      default -> {
      SmartDashboard.putNumber("CURRENT_ZONE", 0);
      }
    }
  }

  private void updateLevelStatus() {
    boolean hasL1 = false;
    boolean[] leftLevels = new boolean[4]; // Index 0 unused (L1 handled separately)
    boolean[] rightLevels = new boolean[4];

    for (CoralScoringCommand cmd : commandQueue) {
      ScoringHeight height = cmd.getHeight();
      if (height == ScoringHeight.L1) {
        hasL1 = true;
        continue;
      }

      int levelIndex = height.ordinal() - 1; // Convert L2-L4 to 1-3
      if (cmd.getSide() == ScoringSide.LEFT) {
        leftLevels[levelIndex] = true;
      } else {
        rightLevels[levelIndex] = true;
      }
    }

    // Update SmartDashboard entries
    SmartDashboard.putBoolean("Queued/L1", hasL1);
    SmartDashboard.putBoolean("Queued/Left/L2", leftLevels[0]);
    SmartDashboard.putBoolean("Queued/Left/L3", leftLevels[1]);
    SmartDashboard.putBoolean("Queued/Left/L4", leftLevels[2]);
    SmartDashboard.putBoolean("Queued/Right/L2", rightLevels[0]);
    SmartDashboard.putBoolean("Queued/Right/L3", rightLevels[1]);
    SmartDashboard.putBoolean("Queued/Right/L4", rightLevels[2]);
  }




    /**
     * Determines which side of a line a point lies on
     * @param x Point x coordinate
     * @param y Point y coordinate
     * @param slope Slope of the line
     * @return positive if above/right of line, negative if below/left of line
     */  
    private double getSideOfLine(double x, double y, double slope) {
      if (Double.isInfinite(slope)) {
          // Vertical line case
          return x - BLUECENTERX;
      }
      // y - y1 = m(x - x1) rearranged to get consistent sign
      return (y - BLUECENTERY) - slope * (x - BLUECENTERX);
    }

  public ReefZone getZone(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();

    boolean[] sides = new boolean[3];
    for (int i = 0; i < 3; i++) {
      sides[i] = getSideOfLine(x, y, lineSlopes[i]) > 0;
    }

    if (sides[0]) {  // Right of vertical
            if (sides[1]) {  // Above 60° line
                if (sides[2]) return ReefZone.ZONE_1;
                else return ReefZone.ZONE_2;
            } else {  // Below 60° line
                if (sides[2]) return ReefZone.ZONE_6;
                else return ReefZone.ZONE_5;
            }
        } else {  // Left of vertical
            if (sides[1]) {  // Above 60° line
                if (sides[2]) return ReefZone.ZONE_2;
                else return ReefZone.ZONE_3;
            } else {  // Below 60° line
                if (sides[2]) return ReefZone.ZONE_5;
                else return ReefZone.ZONE_4;
            }
        }
  }

}
