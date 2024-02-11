// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code LauncherSubsystem} class is a subsystem that controls the speed of a launcher using a
 * PID Controller and simple motor feedforward. It uses a CANSparkMax motor and a RelativeEncoder to
 * measure the launcher's speed. The class provides methods to return commands that run the launcher
 * at the specified speed or stop the motor.
 *
 * <p>The LauncherSubsystem class provides a constructor where hardware dependencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of LauncherSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * launcherHardware = new LauncherSubsystem.Hardware(motor, encoder);
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(launcherHardware);
 *
 * // Create a new instance of LauncherSubsystem using default hardware
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(initializeHardware());
 *
 * // Run the launcher at a specific speed
 * Command runLauncherCommand = launcherSubsystem.runLauncher(500.0);
 * runLauncherCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an launcher using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the launcher's state.
 *   - {@code updateLauncherController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runLauncher(double setpoint)}: Returns a Command that runs the launcher at the
 *     defined speed.
 *   - {@code setSetPoint(double goal)}: Set the setpoint for the launcher..
 *   - {@code atSetpoint()}: Returns whether the launcher has reached the set point velocity
 *     within limits.
 *   - {@code enable()}: Enables the PID control of the launcher.
 *   - {@code disable()}: Disables the PID control of the launcher.
 *   - {@code getLauncherSpeedRight()}: Returns the launcher position for PID control and logging.
 *   - {@code getLauncherSpeedLeft()}: Returns the launcher position for PID control and logging.
 *   - {@code getLauncherVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the launcher.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the launcher's
 *     position.
 *   - {@code private PIDController launcherController}: The PID controller used to
 *     control the launcher's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean launcherEnabled}: A flag indicating whether the launcher is enabled.
 *   - {@code private double launcherVoltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class LauncherSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the launcher subsystem. */
  public static class Hardware {
    CANSparkMax launcherMotorRight;
    CANSparkMax launcherMotorLeft;
    RelativeEncoder launcherEncoderRight;
    RelativeEncoder launcherEncoderLeft;

    public Hardware(
        CANSparkMax launcherMotorRight,
        CANSparkMax launcherMotorLeft,
        RelativeEncoder launcherEncoderRight,
        RelativeEncoder launcherEncoderLeft) {
      this.launcherMotorRight = launcherMotorRight;
      this.launcherMotorLeft = launcherMotorLeft;
      this.launcherEncoderRight = launcherEncoderRight;
      this.launcherEncoderLeft = launcherEncoderLeft;
    }
  }

  private final CANSparkMax launcherMotorRight;
  private final CANSparkMax launcherMotorLeft;
  private final RelativeEncoder launcherEncoderRight;
  private final RelativeEncoder launcherEncoderLeft;

  private PIDController launcherleftController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);
  private PIDController launcherrightController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.LAUNCHER_KS_VOLTS.getValue(),
          LauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue(),
          LauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue());

  private double pidleftOutput = 0.0;
  private double pidrightOutput = 0.0;
  private double newleftFeedforward = 0;
  private double newrightFeedforward = 0;
  private boolean launcherEnabled;
  private double launcherVoltageLeftCommand = 0.0;
  private double launcherVoltageRightCommand = 0.0;

  /** Create a new LauncherSubsystem controlled by a Profiled PID COntroller . */
  public LauncherSubsystem(Hardware launcherHardware) {
    this.launcherMotorRight = launcherHardware.launcherMotorRight;
    this.launcherMotorLeft = launcherHardware.launcherMotorLeft;
    this.launcherEncoderRight = launcherHardware.launcherEncoderRight;
    this.launcherEncoderLeft = launcherHardware.launcherEncoderLeft;

    initializeLauncher();
  }

  private void initializeLauncher() {

    RobotPreferences.initPreferencesArray(LauncherConstants.getLauncherPreferences());

    initLauncherMotor();
    initLauncherEncoder();

    // Set tolerances that will be used to determine when the launcher is at the goal velocity.
    launcherleftController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);
    launcherrightController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);

    disableLauncher();
  }

  private void initLauncherMotor() {
    launcherMotorRight.restoreFactoryDefaults();
    launcherMotorLeft.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    launcherMotorRight.clearFaults();
    launcherMotorLeft.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    launcherMotorRight.setIdleMode(IdleMode.kBrake);
    launcherMotorLeft.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Launcher motor firmware version:" + launcherMotorRight.getFirmwareString());
    DataLogManager.log("Launcher motor firmware version:" + launcherMotorLeft.getFirmwareString());
  }

  private void initLauncherEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // launcher position will only be correct if the launcher is in the starting rest position when
    // the subsystem is constructed.
    launcherEncoderRight.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderLeft.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderRight.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderLeft.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
  }

  /**
   * Create hardware devices for the launcher subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax launcherMotorRight =
        new CANSparkMax(LauncherConstants.RIGHT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax launcherMotorLeft =
        new CANSparkMax(LauncherConstants.LEFT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder launcherEncoderRight = launcherMotorRight.getEncoder();
    RelativeEncoder launcherEncoderLeft = launcherMotorLeft.getEncoder();
    launcherMotorRight.setInverted(true);
    return new Hardware(
        launcherMotorRight, launcherMotorLeft, launcherEncoderLeft, launcherEncoderRight);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Launcher Enabled", launcherEnabled);
    SmartDashboard.putNumber("Launcher Setpoint", launcherleftController.getSetpoint());
    SmartDashboard.putNumber("Launcher Setpoint", launcherrightController.getSetpoint());
    SmartDashboard.putNumber("Launcher Speed", launcherEncoderRight.getVelocity());
    SmartDashboard.putNumber("Launcher Speed", launcherEncoderLeft.getVelocity());
    //    SmartDashboard.putNumber("Launcher left Voltage", launcherVoltageLeftCommand);
    //  SmartDashboard.putNumber("Launcher right Voltage", launcherVoltageRightCommand);
    SmartDashboard.putNumber("Launcher Current", launcherMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Current", launcherMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Launcher left Feedforward", newleftFeedforward);
    SmartDashboard.putNumber("Launcher right Feedforward", newrightFeedforward);
    SmartDashboard.putNumber("Launcher left PID output", pidleftOutput);
    SmartDashboard.putNumber("Launcher right PID output", pidrightOutput);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateLauncherController() {
    if (launcherEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the launcher at the desired speed. Store the individual values for logging.
      pidrightOutput = launcherrightController.calculate(getLauncherSpeedRight());
      pidleftOutput = launcherleftController.calculate(getLauncherSpeedLeft());
      newleftFeedforward = feedforward.calculate(launcherleftController.getSetpoint());
      newrightFeedforward = feedforward.calculate(launcherrightController.getSetpoint());
      launcherVoltageLeftCommand = pidleftOutput + newleftFeedforward;
      launcherVoltageRightCommand = pidrightOutput + newrightFeedforward;

    } else {
      // If the launcher isn't enabled, set the motor command to 0. In this state the launcher
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidleftOutput = 0;
      pidrightOutput = 0;
      newleftFeedforward = 0;
      newrightFeedforward = 0;
      launcherVoltageLeftCommand = 0;
      launcherVoltageRightCommand = 0;
    }
    launcherMotorRight.setVoltage(launcherVoltageRightCommand);
    launcherMotorLeft.setVoltage(launcherVoltageLeftCommand);
  }

  /** Returns a Command that runs the launcher at the defined speed. */
  public Command runLauncher(double setpoint) {
    return new FunctionalCommand(
        () -> setLauncherSetPoint(setpoint),
        this::updateLauncherController,
        interrupted -> disableLauncher(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the launcher. The PIDController drives the launcher to this speed and
   * holds it there.
   */
  private void setLauncherSetPoint(double setpoint) {
    launcherleftController.setSetpoint(setpoint);
    launcherrightController.setSetpoint(-setpoint);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableLauncher();
  }

  /** Returns whether the launcher has reached the set point speed within limits. */
  public boolean launcherAtSetpoint() {
    return launcherleftController.atSetpoint();
  }

  /**
   * Sets up the PID controller to run the launcher at the defined setpoint speed. Preferences for
   * tuning the controller are applied.
   */
  private void enableLauncher() {

    // Don't enable if already enabled since this may cause control transients
    if (!launcherEnabled) {
      loadPreferences();

      // Reset the PID controller to clear any previous state
      launcherleftController.reset();
      launcherrightController.reset();
      launcherEnabled = true;

      DataLogManager.log(
          "Launcher Enabled - kP="
              + launcherleftController.getP()
              + " kI="
              + launcherleftController.getI()
              + " kD="
              + launcherleftController.getD()
              + " Setpoint="
              + launcherleftController.getSetpoint()
              + " CurSpeedRight="
              + getLauncherSpeedRight()
              + " CurSpeedLeft="
              + getLauncherSpeedLeft());
    }
  }

  /**
   * Disables the PID control of the launcher. Sets motor output to zero. NOTE: In this state the
   * launcher will slow down until it stops. Motor EMF braking will cause it to slow down faster if
   * that mode is used.
   */
  public void disableLauncher() {

    // Clear the enabled flag and update the controller to zero the motor command
    launcherEnabled = false;
    updateLauncherController();

    // Cancel any command that is active
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log("Launcher Disabled CurSpeedRight=" + getLauncherSpeedRight());
    DataLogManager.log("Launcher Disabled CurSpeedLeft=" + getLauncherSpeedLeft());
  }

  /** Returns the launcher speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedRight() {
    return launcherEncoderRight.getVelocity();
  }

  public double getLauncherSpeedLeft() {
    return launcherEncoderLeft.getVelocity();
  }

  /** Returns the launcher motor commanded voltage. */
  public double getLauncherVoltageCommand() {
    return launcherVoltageLeftCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    launcherleftController.setP(LauncherConstants.LAUNCHER_KP.getValue());
    launcherrightController.setP(LauncherConstants.LAUNCHER_KP.getValue());

    // Read Preferences for Feedforward and create a new instance
    double staticGain = LauncherConstants.LAUNCHER_KS_VOLTS.getValue();
    double velocityGain = LauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue();
    double accelerationGain = LauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue();
    feedforward = new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    launcherMotorRight.close();
    launcherMotorLeft.close();
  }
}
