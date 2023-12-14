// romiGyroDrivPID - D                             CONSTANT.J

package frc.robot;

public final class Constant {   
	  // setpoint = a distance for an auto cmd to drive  
	  public static final double kDistP = 0.1;
	  public static final double kDistI = 0.004;
	  public static final double kDistD = 0.00;

  // worked well to stabiliz auto and teleop straight drive 
      public static final double kStabilP = 0.015;
      public static final double kStabilI = 0.0025;
      public static final double kStabilD = 0.0;

  // can be erratic, needed fine tuning of k_'s, still erratic 
  // hot edit not reliable; sim restart could help  
      public static final double kTurnP = 0.002;
      public static final double kTurnI = 0.0005;
      public static final double kTurnD = 0.00;
  
      // public static final double kMaxTurnVeloc = 20;
      // public static final double kMaxTurnAcceler = 20;
  
      // public static final double kTurnTolerDeg = 2;
      // public static final double kTurnTolerVeloc = 10; 
      // deg per second
      
  // inner class example could go inside a subsys class too
    // public static final class OIConstants {
    // public static final int kDrivXboxPort = 0;
  
 }  // end constant