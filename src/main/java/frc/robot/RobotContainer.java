// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
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
  // The Blinkin
  private BlinkinSubsystem blinkin = new BlinkinSubsystem(new PWM(Constants.BLINKIN_PORT));

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Setup the Autonomous mode command chooser
    setupAutoChooser();

    blinkin.enable();

    this.robotDrive.setDefaultCommand(this.robotDrive.getDriveCommand(driverController));
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
        .onTrue(new InstantCommand(this.robotDrive::setCrawlSpeed))
        .onFalse(new InstantCommand(this.robotDrive::setNormalSpeed));

    // Move the arm to the low position when the 'A' button is pressed on the operator's controller.
    operatorController
        .a()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION_RADS)
                .andThen(robotArm::disable)
                .withName("Arm: Move to Forward Position"));

    // Move the arm to the high position when the 'B' button is pressed on the operator's
    // controller.
    operatorController
        .b()
        .onTrue(
            robotArm
                .moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION_RADS)
                .andThen(robotArm::disable)
                .withName("Arm: Move to Back Position"));

    // Shift position down a small amount when the POV Down is pressed on the operator's controller.
    operatorController.povDown().onTrue(robotArm.shiftDown());

    // Shift position up a small amount when the POV Down is pressed on the operator's controller.
    operatorController.povUp().onTrue(robotArm.shiftUp());

    // Disable the arm controller when the 'X' button is pressed on the operator's controller.
    // NOTE: This is intended for initial arm testing and should be removed in the final robot
    // to prevent accidental disable resulting in lowering of the arm.
    operatorController.x().onTrue(Commands.runOnce(robotArm::disable));

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

    // This command runs the launcher at high speed to launch a note into the speaker, then run
    // the intake when the launcher is up to speed.
    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                    robotLauncher.runLauncherSpeaker(),
                    Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))
                .withTimeout(5.0)
                .withName("Intake-Launcher: autoLaunch"));

    // This command runs the launcher at low speed to launch a note into the amp, then run intake
    // when the launcher is up to speed.
    operatorController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                    robotLauncher.runLauncherAmp(),
                    Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))
                .withName("Intake-Launcher: autoLaunchAmp"));

    // Run the intake forward when the right bumper is pressed.
    operatorController
        .rightBumper()
        .whileTrue(robotIntake.runForward().withName("Intake: Run Forward"));

    // Run the intake in reverse when the left bumper is pressed.
    operatorController
        .leftBumper()
        .whileTrue(robotIntake.runReverse().withName("Intake: Run Reverse"));

    // Run the intake until a note is loaded and move the arm back when the Y button is pressed and
    // held.
    operatorController
        .y()
        .whileTrue(
            robotIntake
                .loadNote()
                .andThen(robotArm.moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION_RADS))
                .andThen(robotArm::disable)
                .withName("Intake: Load Note"));
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

  /** Get the drive command from the drive subsystem. */
  public Command getTeleopDriveCommand() {
    return robotDrive.getDriveCommand(driverController);
  }

  /** Setup the options for the Autonomous mode command chooser. */
  private void setupAutoChooser() {

    autoChooser.setDefaultOption("Nothing", "Nothing");
    autoChooser.addOption("Taxi", "DriveStraight");
    autoChooser.addOption("Launch", "OnlyLaunch");
    autoChooser.addOption("Launch and Taxi Straight", "LaunchAndTaxiStraight");
    autoChooser.addOption("Launch Right and Taxi", "LaunchRightAndTaxi");
    autoChooser.addOption("Launch Right and Taxi Far", "LaunchAndTaxiFarRight");
    autoChooser.addOption("Launch Left and Taxi", "LaunchLeftAndTaxi");
    autoChooser.addOption("Launch Left and Taxi Far", "LaunchLeftAndTaxiFar");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    switch (autoChooser.getSelected()) {
      case "DriveStraight":
        // Drive forward slowly until the robot moves 1 meter
        return robotDrive
            .driveDistanceCommand(1.0, 0.1, 0.0)
            .withTimeout(5)
            .withName("Drive Forward 1m");

      case "Launch":
        // Launch a note into the speaker
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))))
            .withName("Launch into speaker");

      case "LaunchAndTaxiStraight":
        // Launch a note then Drive forward slowly until the robot moves a set distance
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))),
                robotDrive.driveDistanceCommand(1.0, 0.1, 0.0))
            .withName("Launch and Drive Forward");

      case "LaunchRightAndTaxi":
        // Start angled right and launch a note then curve left slowly until the robot is straight
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))),
                robotDrive.driveDistanceCommand(0.5, 0.2, 0.2),
                robotDrive.driveDistanceCommand(1.775, 0.15, 0.0))
            .withName("Launch Right and Drive");

      case "LaunchAndTaxiFarRight":
        // Start angled right and launch a note then drive straight to end on right of the field
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))),
                robotDrive.driveDistanceCommand(3.0, 0.15, 0.01))
            .withName("Launch Right and Drive Far");

      case "LaunchLeftAndTaxi":
        // Start angled left and launch a note then curve right slowly until the robot is straight
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))),
                robotDrive.driveDistanceCommand(0.5, 0.2, -0.2),
                robotDrive.driveDistanceCommand(1.775, 0.15, 0.0))
            .withName("Launch Left and Drive");

      case "LaunchLeftAndTaxiFar":
        // Start angled left and launch a note then drive straight to end on Left of the field
        return Commands.sequence(
                Commands.race(
                    robotLauncher.runLauncherSpeaker().withTimeout(4.0),
                    (Commands.waitUntil(robotLauncher::launcherAtSetpoint)
                        .andThen(robotIntake.runReverse()))),
                robotDrive.driveDistanceCommand(3.0, 0.15, -0.01))
            .withName("Launch Left and Drive Far");

      default:
        return new PrintCommand("No Auto Selected");
    }
  }

  /**
   * Use this to get the chooser for the Autonomous mode command.
   *
   * @return a reference to the chooser for the autonomous command
   */
  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
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
   * Use this to get the Blinkin to set the LEDs.
   *
   * @return Blinkin subsystem.
   */
  public BlinkinSubsystem getBlinkin() {
    return this.blinkin;
  }

  /**
   * Use this to get the Arm Subsystem.
   *
   * @return a reference to the arm subsystem
   */
  public ArmSubsystem getArmSubsystem() {
    return robotArm;
  }

  /**
   * Use this to get the Drivetrain Subsystem.
   *
   * @return a reference to the Drivetrain Subsystem
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
