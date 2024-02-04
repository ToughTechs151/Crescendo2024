// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // First we do things that are in all Robots.
  private PowerDistribution pdp = new PowerDistribution();
  // The driver's controller
  private CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // The operator's controller
  private CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  private double normalSpeedMax = 1.0;
  private double crawlSpeedMax = 0.5;

  // Now all the subsystems.
  // The Arm.
  private final ArmSubsystem robotArm = new ArmSubsystem(ArmSubsystem.initializeHardware());
  // The Climber.
  private final ClimberSubsystem robotClimber =
      new ClimberSubsystem(ClimberSubsystem.initializeHardware());
  // The Drive.
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  // The Launcher.
  private final LauncherSubsystem robotLauncher =
      new LauncherSubsystem(LauncherSubsystem.initializeHardware());
  // The Intake.
  private final IntakeSubsystem robotIntake =
      new IntakeSubsystem(IntakeSubsystem.initializeHardware());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    this.robotDrive.setDefaultCommand(getDriveCommand());

    // Disable the internal drive deadband so it can be applied directly to the joystick
    this.robotDrive.setMaxOutput(normalSpeedMax);
  }

  /** Setup the drive command using the tunable settings. */
  public Command getDriveCommand() {

    boolean squareInputs = SmartDashboard.getBoolean("Square Inputs", true);
    boolean enableBrake = SmartDashboard.getBoolean("Enable Brake", true);
    double deadband = SmartDashboard.getNumber("Deadband", 0.05);
    double turnFactor = SmartDashboard.getNumber("Turning Factor", 1.0);
    double slewLimitSpeed = SmartDashboard.getNumber("Slew Limit Speed", 100.0);
    double slewLimitTurn = SmartDashboard.getNumber("Slew Limit Turn", 100.0);
    normalSpeedMax = SmartDashboard.getNumber("Normal Speed", 1.0);
    crawlSpeedMax = SmartDashboard.getNumber("Crawl Speed", 0.5);

    // Slew rate limiters for joystick inputs (units/sec). For example if the limit=2.0, the input
    // can go from 0 to 1 in 0.5 seconds.
    SlewRateLimiter speedLimiter = new SlewRateLimiter(slewLimitSpeed);
    SlewRateLimiter turnLimiter = new SlewRateLimiter(slewLimitTurn);

    // A split-stick arcade command, with forward/backward controlled by the left hand, and turn
    // rate controlled by the right. A deadband is applied to both joysticks to avoid creep due to
    // off calibration. Slew rate limits are applied to speed and turn controls. An additional
    // factor is used
    // to desensitize turning. The motor brake mode is set when the command is initialized.
    return new FunctionalCommand(
            () -> this.robotDrive.setBrakeMode(enableBrake),
            () ->
                this.robotDrive.arcadeDrive(
                    -speedLimiter.calculate(
                        MathUtil.applyDeadband(this.driverController.getLeftY(), deadband)),
                    -turnFactor
                        * turnLimiter.calculate(
                            MathUtil.applyDeadband(this.driverController.getRightX(), deadband)),
                    squareInputs),
            interrupted -> {},
            () -> false,
            this.robotDrive)
        .withName("Arcade");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at low speed when the right bumper is held, otherwise allow normal speed.
    driverController
        .rightBumper()
        .onTrue(new InstantCommand(() -> this.robotDrive.setMaxOutput(crawlSpeedMax)))
        .onFalse(new InstantCommand(() -> this.robotDrive.setMaxOutput(normalSpeedMax)));

    // Move the arm to the low position when the 'A' button is pressed on the operator's controller.
    operatorController
        .a()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION)
                .withName("Arm: Move to Forward Position"));

    // Move the arm to the high position when the 'B' button is pressed on the operator's
    // controller.
    operatorController
        .b()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION)
                .withName("Arm: Move to Back Position"));

    // Shift position down a small amount when the POV Down is pressed on the operator's controller.
    operatorController.povDown().onTrue(robotArm.shiftDown());

    // Shift position up a small amount when the POV Down is pressed on the operator's controller.
    operatorController.povUp().onTrue(robotArm.shiftUp());

    // Disable the arm controller when the 'X' button is pressed on the operator's controller.
    // NOTE: This is intended for initial arm testing and should be removed in the final robot
    // to prevent accidental disable resulting in lowering of the arm.
    operatorController.x().onTrue(Commands.runOnce(robotArm::disable));

    // Put the tunable variables on the dashboard with default values
    SmartDashboard.putBoolean("Enable Brake", true);
    SmartDashboard.putBoolean("Square Inputs", true);
    SmartDashboard.putNumber("Deadband", 0.05);
    SmartDashboard.putNumber("Turning Factor", 1.0);
    SmartDashboard.putNumber("Slew Limit Speed", 100.0);
    SmartDashboard.putNumber("Slew Limit Turn", 100.0);
    SmartDashboard.putNumber("Normal Speed", 1.0);
    SmartDashboard.putNumber("Crawl Speed", 0.5);

    // Move the climber to the low position when the 'A' button is pressed.
    driverController
        .a()
        .onTrue(
            robotClimber
                .moveToPosition(Constants.ClimberConstants.CLIMBER_EXTEND_POSITION_METERS)
                .withName("Climber: Lower"));

    // Move the climber to the high position when the 'Y' button is pressed.
    driverController
        .y()
        .onTrue(
            robotClimber
                .moveToPosition(Constants.ClimberConstants.CLIMBER_RETRACT_POSITION_METERS)
                .withName("Climber: Pull Up"));

    // Disable the climber controller when the 'X' button is pressed.
    driverController.x().onTrue(Commands.runOnce(robotClimber::disable));

    // Run the launcher at the defined speed while the right trigger is held.
    operatorController
        .rightTrigger()
        .whileTrue(
            robotLauncher
                .runLauncher(Constants.LauncherConstants.LAUNCHER_FULL_SPEED)
                .withName("Launcher: Run Full Speed"));

    // Start the intake when the left bumper is pressed.
    operatorController
        .leftBumper()
        .onTrue(
            robotIntake
                .runIntake(Constants.IntakeConstants.INTAKE_COMMAND_VOLTS)
                .withName("Intake: Run"));

    // Stop the intake when the right bumper is pressed.
    operatorController.rightBumper().onTrue(robotIntake.stopIntake().withName("Intake: Stop"));
  }

  /**
   * Disables all subsystems. This should be called on robot disable to prevent integrator windup in
   * subsystems with PID controllers. It also allows subsystems to setup disabled state so
   * simulation matches RoboRio behavior. Commands are canceled at the Robot level.
   */
  public void disableSubsystems() {
    robotArm.disable();
    robotDrive.disable();
    robotClimber.disable();
    robotIntake.disableIntake();
    robotLauncher.disableLauncher();
    DataLogManager.log("disableSubsystems");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Drive forward slowly unitl the robot moves 1 meter
    return new RunCommand(() -> this.robotDrive.arcadeDrive(0.3, 0.0, false), this.robotDrive)
        .until(() -> robotDrive.getAverageDistanceMeters() > 1.0)
        .withTimeout(5)
        .andThen(() -> this.robotDrive.tankDriveVolts(0, 0))
        .withName("Drive Forward 1m");
  }

  /**
   * Use this to get the PDP for data logging.
   *
   * @return The PowerDistribution module.
   */
  public PowerDistribution getPdp() {
    return this.pdp;
  }

  /**
   * Use this to get the Arm Subsystem.
   *
   * @return the command to run in autonomous
   */
  public ArmSubsystem getArmSubsystem() {
    return robotArm;
  }

  /**
   * Use this to get the Drivetrain Subsystem.
   *
   * @return the Drivetrain Subsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return robotDrive;
  }

  /**
   * Use this to get the Climber Subsystem.
   *
   * @return a reference to the Climber Subsystem
   */
  public ClimberSubsystem getClimberSubsystem() {
    return robotClimber;
  }

  /**
   * Use this to get the Intake Subsystem.
   *
   * @return a reference to the Intake Subsystem
   */
  public IntakeSubsystem getIntakeSubsystem() {
    return robotIntake;
  }

  /**
   * Use this to get the Launcher Subsystem.
   *
   * @return a reference to the Launcher Subsystem
   */
  public LauncherSubsystem getLauncherSubsystem() {
    return robotLauncher;
  }
}
