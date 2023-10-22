// RomigyroDrivPID - D                    Robot.j

// v. C -- WPI PIDcontroller for drive & turn, PWM motor, cmd/subsys framewk.
// v. D (this) - simplify to flat framework, no cmd or subsys, all in robot.j
// --> works OK, PID controllers for straight driving and auto turning to
// some angle. Turns still erratic. autoPeriodic does not end properly. RC
// left here for reference, does nothing; other unused classes deleted.

// For live vision, attach camera to any pi port; its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg stream,
// and (when Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import frc.robot.Constant;
import frc.robot.OnBoardIO.ChannelMode;
import frc.robot.sensors.RomiGyro;

/**
 * The VM is configured to automatically run this class, calling the
 * functions corresponding to each mode, as described in TimedRobot docs.
 * -- very little here specific to any one robot --
 */
public class Robot extends TimedRobot {

  // instance joystick @ 0 --assumes controller plugged into USB 0
  private final XboxController m_controller = new XboxController(0);

  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  private final double kCountsPerRevolution = 1440.0;
  private final double kWheelDiameterInch = 2.75591; // 70 mm
  // private double maxFactor = 1.0; // speed multiplier possible

  // Romi has the left and right motors on PWM channels 0 and 1
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  /// DD class has aD() method, default deadband 0.02, squares inputs]
  public final DifferentialDrive m_Drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // The Romi's onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  public static Encoder m_leftEncoder = new Encoder(4, 5);
  public static Encoder m_rightEncoder = new Encoder(6, 7);

  // instance the RomiGyro
  public static RomiGyro m_gyro = new RomiGyro();

  // allows SmartDashboard to pick autonomous routine by name
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected; // returned by SmdDash for use in autoInit

  // PID control to drive straight...
  private final PIDController piDriv = new PIDController(Constant.kStabilP, Constant.kStabilI, Constant.kStabilD);
  // ... turn to some angle
  private final PIDController pidTurn = new PIDController
              (Constant.kTurnP, Constant.kTurnI, Constant.kTurnD);

     /*
   * roboInit runs when the robot is first started and does all init's of
   * this robot's specifics (things done in RC in subsys framework)
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

    m_chooser.setDefaultOption("Drive+Turn180", "DRIV&TURN");
    m_chooser.addOption("turn180", "TURN180");
    SmartDashboard.putData("AutoSelect ", m_chooser);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_gyro.reset();

    // the delta tolerance ensures the robot is stable at the
    // setpoint before it's counted as reaching the reference
    piDriv.setTolerance(1, 2);

    pidTurn.enableContinuousInput(-180, 180);
    pidTurn.setTolerance(2, 2);
  } // end robotInit()

  // This function is called every robot packet, no matter the mode.
  // Use for things that you want run during all modes like diagnostics.
  // This runs after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() { // normally ... in cmd/subsys paradigm:
    // Calls the Scheduler <-- this is responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Must be here for anything in the Command-based framework to work.
    // ... in flat framework will run essential stuff regardless of mode
    if (m_controller.getRawButton(1)) // [xbox button A]
      m_gyro.reset();

    SmartDashboard.putNumber("Z axis Rot", m_gyro.getAngleZ());

    // in teleOp, this forces drive straight, no manual turn possible
    if (m_controller.getRawButton(6)) { // Rt bumper press
      // double gyroAdjust = m_gyro.getAngleZ() * Constant.kStabilP;
      m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.6,
          -piDriv.calculate(m_gyro.getAngleZ(), 0));
    }

  } // end robotPeriodic

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override // incessant (-) drift, so to reset manually in disabled,
            // you can press button 1 [xbox button A]: --try in rP first
  public void disabledPeriodic() {
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
  } // end autoInit

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) { // sequence built by changing this var
      case "DRIV&TURN":
        // drive strait 36", stop when there, change autoselec to turn180
        // double gyroAdjust = m_gyro.getAngleZ() * kStabilD;
        m_Drive.arcadeDrive(0.6, -piDriv.calculate(m_gyro.getAngleZ(), 0));
        if (m_leftEncoder.getDistance() + m_rightEncoder.getDistance() / 2 >= 36) {
          m_Drive.arcadeDrive(0, 0);
          m_gyro.reset();
          m_autoSelected = "TURN180";
          break;
        }
        break;
      case "TURN180":
        // Pipe output to turn robot in place, should turn CW for + setpoint
        m_Drive.arcadeDrive(0, -pidTurn.calculate(m_gyro.getAngleZ(), -179));
        if (pidTurn.atSetpoint()) {
          m_Drive.arcadeDrive(0, 0);
          m_autoSelected = "end";
        }
        break;
      case "end":
         CommandScheduler.getInstance().cancelAll();
         break;
      default:
        break;
    } // when done autoSelect --> "end" which --> default and does nothing
      // but aP still does not seem to end as it should; need some cmd to end
      // like CommandScheduler.getInstance().cancelAll();
  } // end autoPeriod

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want the
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoSelected != null) { // if auto never activated, don't need resets
      // m_autoSelected.cancel(); // can't cancel a string
      m_leftEncoder.reset();
      m_rightEncoder.reset();
      m_gyro.reset();
    } // end if
  } // end teleInit

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { // rt bumper button hold activates
    // gyro mode to drive straight in teleop, otherwise drifts
    m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.6,
        -m_controller.getRightX() * 0.4);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end class