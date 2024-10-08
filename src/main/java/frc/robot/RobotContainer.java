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

    // ---------- Driver Controller ----------
    // Drive at low speed when the right bumper is held, otherwise allow normal speed.
    driverController
        .rightBumper()
        .onTrue(new InstantCommand(this.robotDrive::setCrawlSpeed))
        .onFalse(new InstantCommand(this.robotDrive::setNormalSpeed));

    // Unlock the pins and then extend the climber arms when the 'Y' button is pressed. A fixed
    // voltage command releases tension while pulling the pins out.
    driverController
        .y()
        .onTrue(
            Commands.sequence(
                    Commands.race(
                        robotClimber.commandVoltage(),
                        Commands.sequence(
                            Commands.waitSeconds(1.0),
                            Commands.runOnce(() -> robotClimber.setRelay(false, true)),
                            Commands.waitSeconds(3.0),
                            Commands.runOnce(() -> robotClimber.setRelay(false, false)))),
                    robotClimber.moveToPosition(
                        Constants.ClimberConstants.CLIMBER_EXTEND_POSITION_METERS),
                    Commands.runOnce(robotClimber::disable))
                .withName("Climber: Extend"));

    // Retract the climber arms when the 'A' button is pressed. Lock the pins when fully retracted
    // and then disable.
    driverController
        .a()
        .onTrue(
            Commands.sequence(
                    robotClimber.moveToPosition(
                        Constants.ClimberConstants.CLIMBER_RETRACT_POSITION_METERS),
                    Commands.race(
                        robotClimber.holdPosition(),
                        Commands.sequence(
                            Commands.runOnce(() -> robotClimber.setRelay(true, false)),
                            Commands.waitSeconds(3.0),
                            Commands.runOnce(() -> robotClimber.setRelay(false, false)))),
                    Commands.runOnce(robotClimber::disable))
                .withName("Climber: Retract"));

    // Disable the climber controller when the 'X' button is pressed.
    driverController.x().onTrue(Commands.runOnce(robotClimber::disable));

    // Command the climber relays to forward/lock position when POV Up is pressed, and then turn
    // relay off when button is released.
    driverController
        .povUp()
        .whileTrue(
            Commands.startEnd(
                    () -> robotClimber.setRelay(true, false),
                    () -> robotClimber.setRelay(false, false))
                .withName("Relay Forward"));

    // Command the climber relays to reverse/unlock position when POV Down is pressed, and then turn
    // relay off when button is released.
    driverController
        .povDown()
        .whileTrue(
            Commands.startEnd(
                    () -> robotClimber.setRelay(false, true),
                    () -> robotClimber.setRelay(false, false))
                .withName("Relay Reverse"));

    // Command the climber motors to a set voltage while the B button is pressed, and set to 0
    // when released.
    driverController.b().whileTrue(robotClimber.commandVoltage());

    // ---------- Operator Controller ----------
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

    // Disable the arm controller when the 'X' button is pressed on the operator's controller.
    // NOTE: This is intended for initial arm testing and should be removed in the final robot
    // to prevent accidental disable resulting in lowering of the arm.
    operatorController.x().onTrue(Commands.runOnce(robotArm::disable));

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
    autoChooser.addOption("2 Note Center", "2NoteCenter");
    autoChooser.addOption("2 Note Left", "2NoteLeft");
    autoChooser.addOption("2 Note Right", "2NoteRight");
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
            .driveForwardCommand(1.0, 0.1, 0.0)
            .withTimeout(5)
            .withName("Drive Forward 1m");

      case "OnlyLaunch":
        // Launch a note into the speaker
        return launcherSequence().withName("Launch into speaker");

      case "LaunchAndTaxiStraight":
        // Launch a note then Drive forward slowly until the robot moves a set distance
        return Commands.sequence(launcherSequence(), robotDrive.driveForwardCommand(1.0, 0.1, 0.0))
            .withName("Launch and Drive Forward");

      case "LaunchRightAndTaxi":
        // Start angled right and launch a note then curve left slowly until the robot is straight
        return Commands.sequence(
                launcherSequence(),
                robotDrive.driveForwardCommand(0.5, 0.2, 0.2),
                robotDrive.driveForwardCommand(1.775, 0.15, 0.0))
            .withName("Launch Right and Drive");

      case "LaunchAndTaxiFarRight":
        // Start angled right and launch a note then drive straight to end on right of the field
        return Commands.sequence(
                launcherSequence(), robotDrive.driveForwardCommand(3.0, 0.15, 0.01))
            .withName("Launch Right and Drive Far");

      case "LaunchLeftAndTaxi":
        // Start angled left and launch a note then curve right slowly until the robot is straight
        return Commands.sequence(
                launcherSequence(),
                robotDrive.driveForwardCommand(0.5, 0.2, -0.2),
                robotDrive.driveForwardCommand(1.775, 0.15, 0.0))
            .withName("Launch Left and Drive");

      case "LaunchLeftAndTaxiFar":
        // Start angled left and launch a note then drive straight to end on Left of the field
        return Commands.sequence(
                launcherSequence(), robotDrive.driveForwardCommand(3.0, 0.15, -0.01))
            .withName("Launch Left and Drive Far");

      case "2NoteCenter":
        // Launch note, pick up a second note, drive back and launch
        return Commands.sequence(
                launcherSequence(),
                robotArm
                    .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION_RADS)
                    .andThen(robotArm::disable),
                loadNote(1.0),
                robotDrive.driveReverseCommand(0.1, 0.2, 0.0).withTimeout(3.0),
                launcherSequence())
            .withName("2 Note Center");

      case "2NoteLeft":
        // Launch note from left side, pick up a second note, drive back and launch
        return Commands.sequence(
                launcherSequence(),
                robotDrive.driveForwardCommand(0.5, 0.2, -0.2),
                robotArm
                    .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION_RADS)
                    .andThen(robotArm::disable),
                loadNote(2.0),
                robotDrive.driveReverseCommand(0.6, 0.2, 0.0),
                robotDrive.driveReverseCommand(0.0, 0.2, 0.2).withTimeout(1.5),
                launcherSequence())
            .withName("2 Note Left");

      case "2NoteRight":
        // Launch note from right side, pick up a second note, drive back and launchgit
        return Commands.sequence(
                launcherSequence(),
                robotDrive.driveForwardCommand(0.5, 0.2, 0.2),
                robotArm
                    .moveToPosition(Constants.ArmConstants.ARM_FORWARD_POSITION_RADS)
                    .andThen(robotArm::disable),
                loadNote(2.0),
                robotDrive.driveReverseCommand(0.6, 0.2, 0.0),
                robotDrive.driveReverseCommand(0.0, 0.2, -0.2).withTimeout(1.5),
                launcherSequence())
            .withName("2 Note Right");

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

  /** Build a Command that runs the launcher and intake to score into to speaker. */
  public Command launcherSequence() {

    return Commands.race(
        robotLauncher.runLauncherSpeaker().withTimeout(2),
        (Commands.waitUntil(robotLauncher::launcherAtSetpoint).andThen(robotIntake.runReverse())));
  }

  /** Build a Command that drive forward while loading a note and then brings the arm back. */
  public Command loadNote(double position) {
    return Commands.sequence(
        Commands.parallel(
                robotIntake
                    .runForward()
                    .until(robotArm::isNoteInsideIntake)
                    .andThen(robotIntake.runForward().withTimeout(0.25)),
                robotDrive.driveForwardCommand(position, 0.2, 0.0))
            .withTimeout(5),
        robotArm
            .moveToPosition(Constants.ArmConstants.ARM_BACK_POSITION_RADS)
            .andThen(robotArm::disable));
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
