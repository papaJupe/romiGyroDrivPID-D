// ROMI GYRO DRIVpid - C                     RobotContainer.j
// not  used in v. D, kept for reference only
package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.sensors.OnBoardIO;
import frc.robot.sensors.OnBoardIO.ChannelMode;

public class RCstub {
  // instance the two subsystems

  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  // NOTE: I/O pin function config possible in web interface; v. base code
  
  // instance joystick @ 0 --assumes controller plugged into USB 0
  // numerous get()s in AD cmd require public stick
  public static final XboxController m_controller = new XboxController(0);

  // allows SmartDashboard to pick autonomous routine
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * CONSTRUCT container for robot: its single method, configBB() sets
   * Drivem_Drive [subsystem's] default Cmd, OperatorInterface (OI) actions,
   * Smart Dashbd Autonomous chooser options.
   * --- i.e. the specifics of this robot
   */
  public RCstub() {
    // Configure joystick button bindings et. al.;  not clear why not
    configureButtonBindings();     // just put things in this block ?
  } // end constructor

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {GenericHID} or one of its subclasses
   * edu.wpi.first.wpilibj.Joystick} or {XboxController}, and then passing
   * it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is ArcadeDrive -- runs unless another command
    // is scheduled over it.(i.e. runs in teleOp unless overridden)
    // [orig.]m_Drive.setDefaultCommand(getArcadeDriveCommand());
    // now less obscure syntax, using simpler constructor
    // m_Drive.setDefaultCommand(new ArcadeDrive(Robot.m_Drive));

    // m_robotDrive.setDefaultCommand(
    // // A split-stick arcade command, with forward/backward controlled
    // // by the left stick, and turning controlled by the right.
    // new RunCommand( () -> m_robotDrive.arcadeDrive(
    // -m_driverController.getLeftY(), -m_driverController.getRightX()),
    // m_robotDrive));

    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whileTrue(new PrintCommand("onbord A Press"))
        .whileFalse(new PrintCommand("onbord A Release"));

    // // stick button A resets Gyro (to 0) [instanced in drive subsystem]
    new JoystickButton(m_controller, 1)
    .onTrue(new InstantCommand(() -> Robot.m_gyro.reset()))
    .onTrue(new PrintCommand("Button A Press"));

    // Stabilize to drive straight with gyro when rt bumper is held
    // new JoystickButton(m_controller, 6)
    //     .whileTrue(
    //         new PIDCommand(
    //             new PIDController(
    //                 Constant.kStabilP,
    //                 Constant.kStabilI,
    //                 Constant.kStabilD),
    //             // feedback comes from the turn rate
    //             Robot.m_gyro::getGyroAngleZ,
    //             // Setpoint is 0
    //             0,
    //             // nb: uses subsys' aD(), bypassing ADcmd; not compatible w/ auto
    //             // Pipe the output to the turning cmd; L still controls speed
    //             output -> m_Drive.arcadeDrive(-m_controller.getLeftY()* 0.6, output),
    //             // Require the robot drive -- needs to be a subsystem instance
    //             Robot.m_Drive));

    // Drive at half speed when the L bumper is held
    // new JoystickButton(m_controller, 5)
    //     .onTrue(new InstantCommand(() -> m_Drive.setMaxOutput(0.6)))
    //     .onFalse(new InstantCommand(() -> m_Drive.setMaxOutput(1)));

    // // Turn to -90 degrees when the 'X' button is pressed, 5 second timeout
    // new JoystickButton(m_controller, 3)
    //     .onTrue(new TurnToAngle(-90, Robot.m_Drive).withTimeout(10));

    // Turn to +90 degrees with a profile when the B button is pressed,
    // with a 5 second timeout
    // new JoystickButton(m_controller, 2)
    //     .onTrue(new TurnToAngleProf(90, m_Drive).withTimeout(5));

    // Setup SmartDashboard options
    // m_chooser.setDefaultOption("Auton Sequen", new AutonSequen(m_Drive));
    //m_chooser.addOption("Auto Turn Gyro 90", new TurnToAngle(90, m_Drive));
    SmartDashboard.putData(m_chooser);

  } // end configBB()

  // passes selected auto command to the scheduling {@link Robot} class.
  // @return the command to run in autonomous
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  } // end get.AutoCmd

} // end class
