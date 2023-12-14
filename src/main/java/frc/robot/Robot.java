// RomigyroDrivPID - D                              Robot.j

// v. D (this) - simplify to flat framework, no cmd or subsys, all in robot.j
// combines PID control x 3: drive distance, drive straight, turn degr.
// work inconsistently, worse than v. C's where I extend PIDCommand. 
// all drive functions good, turns erratic, unpredictable --
// maybe some random error in gyro ? Likely better turns by using
// equal but opposite encoder distances 
//  autoPeriodic does not end properly w/o actual Cmd's--
// Cmd run and canceled does not stop it. aP fails to recognize WaitCmds
// RCstub kept for reference, does nothing; other unused classes deleted.

// v. C -- WPI PIDcontroller for drive & turn, PWM motor, cmd/subsys framewk

// For live vision, attach camera to any pi port; its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg stream,
// and (when Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.sensors.OnBoardIO;
import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.OnBoardIO.ChannelMode;

/**
 * The VM is configured to automatically run this class, calling the
 * functions corresponding to each mode, as described in TimedRobot docs.
 * all robot config set in Robot.j -- normally few specifics here
 */
public class Robot extends TimedRobot {

  // instance joystick @ 0 --assumes controller plugged into USB 0
  private final XboxController m_controller = new XboxController(0);

  private final OnBoardIO m_onboardIO = new OnBoardIO
              (ChannelMode.INPUT, ChannelMode.INPUT);

  private final double kCountsPerRevolution = 1440.0;
  private final double kWheelDiameterInch = 2.75591; // 70 mm
  // private double maxFactor = 1.0; // speed multiplier possible

  // Romi has the left and right motors on PWM channels 0 and 1
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  /// DD class has aD() method, default: deadband 0.02, squares inputs
  public final DifferentialDrive m_Drive = new DifferentialDrive
               (m_leftMotor, m_rightMotor);

  // The Romi's onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  public static Encoder m_leftEncoder = new Encoder(4, 5);
  public static Encoder m_rightEncoder = new Encoder(6, 7);

  // instance the RomiGyro
  public static RomiGyro m_gyro = new RomiGyro();

  // allows SmartDashboard to pick autonomous routine by name
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected; // returned by chooser for use in autoInit

  // PID control to drive straight - setpoint 0
  private final PIDController piDist = new PIDController
                   (Constant.kDistP, Constant.kDistI, Constant.kDistD);

  // PID control to drive straight - setpoint 0
  private final PIDController piDriv = new PIDController
              (Constant.kStabilP, Constant.kStabilI, Constant.kStabilD);
  // ... turn to some angle
  private final PIDController pidTurn = new PIDController
                  (Constant.kTurnP, Constant.kTurnI, Constant.kTurnD);

  private Command stopIt; // attempt to stop autoPeriodic, fails
  private Timer mTimer;
  Boolean goBack = false; // used in autoInit, repeat sequence or not

  /*
   * roboInit runs when the robot is first started and does all init's of
   * this robot's specifics (things normally in RC in cmd/subsys framwk)
   */
  @Override
  public void robotInit() {
    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA /// may need RP to call scheduler
        .whileTrue(new PrintCommand("onbord A Press"))
        .whileFalse(new PrintCommand("onbord A Release"));

    // invert one side of the drivetrain so that positive voltage
    // results in each side moving forward. Depending on how your robot
    // gearbox is constructed, you might have to invert left side instead.
    m_rightMotor.setInverted(true);

    m_chooser.setDefaultOption("turn180", "TURN180");
    m_chooser.addOption("Drive&Turn180", "DRIV&TURN");
    m_chooser.addOption("turn&return", "RETURN");
    SmartDashboard.putData("AutoSelect ", m_chooser);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_gyro.reset();

    stopIt = new InstantCommand(() -> 
             m_Drive.arcadeDrive(0, 0))
            .andThen(new PrintCommand("StoppingAuto"));

    // the delta tolerance ensures the robot is stable at the
    // setpoint before it's counted as reaching the reference
    piDist.setTolerance(2, 3);

    piDriv.setTolerance(2, 3);
    // degree pos. Tol., deg/sec rate Tol.

    pidTurn.enableContinuousInput(-180, 180);
    pidTurn.setTolerance(3, 3); 
  } // end robotInit()       // deg, deg/sec

  // This function is called every robot packet, no matter the mode.
  @Override
  public void robotPeriodic() { // normally, in cmd/subsys paradigm:
    // calls the Scheduler <-- responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Must be here for anything in the Command-based framework to work.
    // ... in flat framework will run essential stuff regardless of mode
    if (m_controller.getRawButton(1)) // [xbox button A]
      m_gyro.reset();

    SmartDashboard.putNumber("Z axis Rot", m_gyro.getAngleZ());

    // in teleOp, this forces drive straight, no manual turn possible
    if (m_controller.getRawButton(6)) { // Rt bumper press
      m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.5,
          -piDriv.calculate(m_gyro.getAngleZ(), 0), false);
    } // probably better control w/ small feedbk numbers if not squared

    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override // incessant (-) drift, so to reset manually in disabled,
            // you can press button 1 [xbox button A]:
  public void disabledPeriodic() { // works from rP, no need to repeat
    // if (m_controller.getRawButton(1))
    // m_gyro.reset();
  }

  // autoInit gets the autonomous command name set by SmartDashbd
  @Override
  public void autonomousInit() {
    // RC got selected auto from SmartDashboard as CMD, I just get String
    m_autoSelected = m_chooser.getSelected();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    goBack = false;

    mTimer = new Timer();
  } // end autoInit

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double distGoal = 36;
    double distNow = (m_leftEncoder.getDistance() +
        m_rightEncoder.getDistance()) / 2;

    switch (m_autoSelected) { // sequence built by changing this var
      case "RETURN":
        goBack = true; // repeat wanted
        m_autoSelected = "DRIV&TURN";
        break;
      case "DRIV&TURN":
        // control drive distance & angle(0) w/ 2 PID controllers
        m_Drive.arcadeDrive(piDist.calculate(distNow, distGoal),
            -piDriv.calculate(m_gyro.getAngleZ(), 0),
            false);
        
        if (piDist.atSetpoint()) {
          m_Drive.arcadeDrive(0, 0);
          // new WaitCommand(4); // fails
          mTimer.start();
        }
        if (mTimer.get() > 5.0) {
          m_gyro.reset();
          mTimer.reset();
          m_autoSelected = "TURN180";
        }
        break;
      case "TURN180":
        // Pipe output to turn bot in place, should turn CCW for (-) setpoint
        m_Drive.arcadeDrive(0, -pidTurn.calculate
                        (m_gyro.getAngleZ(), 179), false); // to square/ not?
        if (pidTurn.atSetpoint()) {
          m_Drive.arcadeDrive(0, 0);
          mTimer.start();

          if (mTimer.get() > 5.0) {
            mTimer.reset();

            if (goBack) {
              m_gyro.reset();
              m_leftEncoder.reset();
              m_rightEncoder.reset();
              m_autoSelected = "DRIV&TURN"; // do it again
              goBack = false; // don't do it again after that
            }
            else { // goBk false, so try to end
              stopIt.schedule();
              mTimer.start();
              m_autoSelected = "endMe";
            }
          } // end timer > 5
        } // end @ set point
        break;




      case "endMe":
       if (mTimer.get() > 0.5)   stopIt.cancel(); 
       // does not stop aP, does not go to teleopInit
        break;
      default:
        break;
    } // when done, autoSelect --> "end" and stopIt() does nothing
      // but print something -- aP still does not seem to end as it should;
      // neither stopIt.cancel() or
      // CommandScheduler.getInstance().cancelAll() ?? work
  } // end autoPeriod

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want 
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoSelected != null) { // if auto never activated, don't do resets
      // m_autoSelected.cancel(); // can't cancel a string
      m_leftEncoder.reset();
      m_rightEncoder.reset();
      m_gyro.reset();
    } // end if
  } // end teleInit

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { // rt bumper button hold activates
    // gyroPID mode to drive straight in teleop, just this drifts to L
    m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.6,
        -m_controller.getRightX() * 0.4);
    // -m_controller.getRawAxis(0) * 0.4);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
} // end class
