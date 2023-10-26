// romiGyroDrivPID - D                             CONSTANT.J

package frc.robot;

/* you can statically import this class (or one of its inner classes),     
 * wherever the constants are needed, to reduce verbosity.
 * most 'base example' values unused by Romi but these few are here
 */
public final class Constant {     
  // worked well to stabiliz auto and teleop straight drive 
      public static final double kStabilP = 0.02;
      public static final double kStabilI = 0.00;
      public static final double kStabilD = 0;

  // can be erratic, needed fine tuning of k_ and output 
  // multiplier; hot edit not reliable; sim restart could help  
      public static final double kTurnP = 0.001;
      public static final double kTurnI = 0.0004;
      public static final double kTurnD = 0.001;
  
      // public static final double kMaxTurnVeloc = 20;
      // public static final double kMaxTurnAcceler = 20;
  
      // public static final double kTurnTolerDeg = 2;
      // public static final double kTurnTolerVeloc = 10; 
      // deg per second
      
  // inner class example
    // public static final class OIConstants {
    // public static final int kDrivXboxPort = 0;
  
 }  // end constant