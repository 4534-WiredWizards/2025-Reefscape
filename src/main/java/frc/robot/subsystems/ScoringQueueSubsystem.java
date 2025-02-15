package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CoralScoringCommand;
import frc.robot.subsystems.drive.Drive;

public class ScoringQueueSubsystem extends SubsystemBase {
  private final Queue<CoralScoringCommand> commandQueue = new LinkedList<>();
  private final Drive swerveDrive;


  public ScoringQueueSubsystem(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }


  public enum ScoringSide {
    LEFT,
    RIGHT
  }

  public enum CoralZones {
    ZONE1,
    ZONE2,
    ZONE3,
    ZONE4,
    ZONE5,
    ZONE6,  
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

  private static final double EPSILON = 1e-6;
  private boolean Zone1Top = false;
  private boolean Zone2Top = false;
  private boolean Zone3Top = false;

  private boolean InZone1 = false;
  private boolean InZone2 = false;
  private boolean InZone3 = false;
  private boolean InZone4 = false; 
  private boolean InZone5 = false;
  private boolean InZone6 = false;


private int calculateZone(VisionSubsystem.getPose pose){  //TODO: Fix
  // Example usage of pose variable

  //account for wierd coordinate system in frc (i made a dumb mistake and this is the simplest way to fix it)
  double x = pose.getY();
  double y = pose.getX();

  double centerX = 0.0;
  double centerY = 0.0;
  double istart = centerX-5;
  double iend = centerX+5;
  

  // Border 1
  for (double i = istart; i < iend; i+= EPSILON) {
    if(y >= centerY){
      Zone1Top = true;
    } else if (x < centerX && y < centerY){
      Zone1Top = false;
    }

    // Border 2
    if(x >= -Math.sqrt(3)*i+centerX || y >= -i/Math.sqrt(3)+centerY){
      Zone2Top = true;
    } else if (x < -Math.sqrt(3)*i+centerX || y < -i/Math.sqrt(3)+centerY){
      Zone2Top = false;
    }

    // Border 3
    if(x >= Math.sqrt(3)*i+centerX || y >= i/Math.sqrt(3)+centerY){
      Zone3Top = true;
    } else if (x < Math.sqrt(3)*i+centerX || y < i/Math.sqrt(3)+centerY){
      Zone3Top = false;
    }

  }
  
//zone 1
  if(Zone2Top == false && Zone3Top == false){
    InZone1 = true;
  } else {
    InZone1 = false;
  }

  //zone 4
  if(Zone2Top == true && Zone3Top == true){
    InZone4 = true;
  } else {
    InZone4 = false;
  }
    
  
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
}
