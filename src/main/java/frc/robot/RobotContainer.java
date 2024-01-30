// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  // The Elevator.
  private final ElevatorSubsystem robotElevator =
      new ElevatorSubsystem(ElevatorSubsystem.initializeHardware());
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

    // Configure default commands
    // Set the default drive command to split-stick tank drive
    this.robotDrive.setDefaultCommand(
        // A split-stick tank command, with left side forward/backward controlled by the left
        // joystick, and right side controlled by the right joystick.
        new RunCommand(
                () ->
                    this.robotDrive.arcadeDrive(
                        -this.driverController.getLeftY(),
                        -this.driverController.getRightX(),
                        true,
                        this.driverController.rightBumper().getAsBoolean()),
                this.robotDrive)
            .withName("Drive: Arcade"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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

    // Move the elevator to the low position when the 'A' button is pressed.
    driverController
        .a()
        .onTrue(
            robotElevator
                .moveToPosition(Constants.ElevatorConstants.ELEVATOR_LOW_POSITION)
                .withName("Elevator: Move to Low Position"));

    // Move the elevator to the high position when the 'Y' button is pressed.
    driverController
        .y()
        .onTrue(
            robotElevator
                .moveToPosition(Constants.ElevatorConstants.ELEVATOR_HIGH_POSITION)
                .withName("Elevator: Move to High Position"));

    // Disable the elevator controller when the 'X' button is pressed.
    driverController.x().onTrue(Commands.runOnce(robotElevator::disable));

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
    robotElevator.disable();
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
    return new RunCommand(
            () -> this.robotDrive.arcadeDrive(0.1, 0.0, false, false), this.robotDrive)
        .until(() -> robotDrive.getAverageDistanceMeters() > 0.5)
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
   * Use this to get the Elevator Subsystem.
   *
   * @return the command to run in autonomous
   */
  public ElevatorSubsystem getElevatorSubsystem() {
    return robotElevator;
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
