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
 * PID Controller and simple motor feedforward. It uses two CANSparkMax motors with RelativeEncoders
 * to measure the launcher's speed. The class provides methods to return commands that run the
 * launcher at the specified speed or stop the motors.
 *
 * <p>The LauncherSubsystem class provides a constructor where hardware dependencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of LauncherSubsystem using specified hardware
 * CANSparkMax motorLeft = new CANSparkMax(1, MotorType.kBrushless);
 * CANSparkMax motorRight = new CANSparkMax(2, MotorType.kBrushless);
 * RelativeEncoder encoderLeft = motorLeft.getEncoder();
 * RelativeEncoder encoderRight = motorRight.getEncoder();
 * launcherHardware = new LauncherSubsystem.Hardware(motorRight, motorLeft, encoderRight,
 *  encoderLeft);
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(launcherHardware);
 *
 * // Create a new instance of LauncherSubsystem using default hardware
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(initializeHardware());
 *
 * // Run the launcher at a specific speed
 * Command runLauncherCommand = launcherSubsystem.runLauncher(5000.0);
 * runLauncherCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an launcher using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Publish telemetry with information about the intake's state.
 *   - {@code updateLauncherController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runLauncher(double setpoint)}: Returns a Command that runs the launcher at the
 *     defined speed.
 *   - {@code setSetPoint(double goal)}: Set the setpoint for the launcher..
 *   - {@code atSetpoint()}: Returns whether the launcher has reached the set point velocity
 *     within limits.
 *   - {@code enable()}: Enables the PID control of the launcher.
 *   - {@code disable()}: Disables the PID control of the launcher.
 *   - {@code getLauncherSpeedRight()}: Returns the right side speed for PID control and logging.
 *   - {@code getLauncherSpeedLeft()}: Returns the left side for PID control and logging.
 *   - {@code getLauncherVoltageCommandLeft()}: Returns the left motor commanded voltage.
 *   - {@code getLauncherVoltageCommandRight()}: Returns the right motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax launcherMotorRight}: The right side motor used to control
 *     the launcher.
 *   - {@code private final CANSparkMax launcherMotorLeftt}: The left side motor used to control
 *     the launcher.
 *   - {@code private final RelativeEncoder launcherEncoderRight}: The right side encoder used to
 *      measure the launcher's speed.
 *   - {@code private final RelativeEncoder launcherEncodeLeft}: The left side encoder used to
 *      measure the launcher's speed.
 *   - {@code private PIDController launcherLeftController}: The PID controller used to
 *     control the left launcher's speed.
 *   - {@code private PIDController launcherRightController}: The PID controller used to
 *     control the right launcher's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidLeftOutput}: The output of the left PID controller.
 *   - {@code private double pidRightOutput}: The output of the right PID controller.
 *   - {@code private double newLeftFeedforward}: The calculated left feedforward value.
 *   - {@code private double newRightFeedforward}: The calculated right feedforward value.
 *   - {@code private boolean launcherEnabled}: A flag indicating whether the launcher is enabled.
 *   - {@code private double launcherVoltageLeftCommand}: The left motor commanded voltage.
 *   - {@code private double launcherVoltageRightCommand}: The right motor commanded voltage.
 * </pre>
 */
public class LauncherSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the launcher subsystem. */
  public static class Hardware {
    CANSparkMax launcherMotorRight;
    CANSparkMax launcherMotorLeft;
    RelativeEncoder launcherEncoderRight;
    RelativeEncoder launcherEncoderLeft;

    /** COnstruct the hardware class to hold the motors and encoders. */
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

  private PIDController launcherLeftController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);
  private PIDController launcherRightController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.LAUNCHER_KS_VOLTS.getValue(),
          LauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue(),
          LauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue());

  private double pidLeftOutput = 0.0;
  private double pidRightOutput = 0.0;
  private double newLeftFeedforward = 0;
  private double newRightFeedforward = 0;
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
    launcherLeftController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);
    launcherRightController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);

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
        launcherMotorRight, launcherMotorLeft, launcherEncoderRight, launcherEncoderLeft);
  }

  /** Publish telemetry with information about the intake's state. */
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Launcher Enabled", launcherEnabled);
    SmartDashboard.putNumber("Launcher Left Setpoint", launcherLeftController.getSetpoint());
    SmartDashboard.putNumber("Launcher Right Setpoint", launcherRightController.getSetpoint());
    SmartDashboard.putNumber("Launcher Right Speed", launcherEncoderRight.getVelocity());
    SmartDashboard.putNumber("Launcher Left Speed", launcherEncoderLeft.getVelocity());
    SmartDashboard.putNumber("Launcher Left Voltage", launcherVoltageLeftCommand);
    SmartDashboard.putNumber("Launcher Right Voltage", launcherVoltageRightCommand);
    SmartDashboard.putNumber("Launcher Right Current", launcherMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Left Current", launcherMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Left Feedforward", newLeftFeedforward);
    SmartDashboard.putNumber("Launcher Right Feedforward", newRightFeedforward);
    SmartDashboard.putNumber("Launcher Left PID output", pidLeftOutput);
    SmartDashboard.putNumber("Launcher Right PID output", pidRightOutput);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateLauncherController() {
    if (launcherEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the launcher at the desired speed. Store the individual values for logging.
      pidRightOutput = launcherRightController.calculate(getLauncherSpeedRight());
      pidLeftOutput = launcherLeftController.calculate(getLauncherSpeedLeft());
      newLeftFeedforward = feedforward.calculate(launcherLeftController.getSetpoint());
      newRightFeedforward = feedforward.calculate(launcherRightController.getSetpoint());
      launcherVoltageLeftCommand = pidLeftOutput + newLeftFeedforward;
      launcherVoltageRightCommand = pidRightOutput + newRightFeedforward;

    } else {
      // If the launcher isn't enabled, set the motor command to 0. In this state the launcher
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidLeftOutput = 0;
      pidRightOutput = 0;
      newLeftFeedforward = 0;
      newRightFeedforward = 0;
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
    launcherLeftController.setSetpoint(setpoint);
    launcherRightController.setSetpoint(-setpoint);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableLauncher();
  }

  /** Returns whether the launcher has reached the set point speed within limits. */
  public boolean launcherAtSetpoint() {
    return (launcherLeftController.atSetpoint() && launcherRightController.atSetpoint());
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
      launcherLeftController.reset();
      launcherRightController.reset();
      launcherEnabled = true;

      DataLogManager.log(
          "Launcher Enabled - kP="
              + launcherLeftController.getP()
              + " kI="
              + launcherLeftController.getI()
              + " kD="
              + launcherLeftController.getD()
              + " Setpoint="
              + launcherLeftController.getSetpoint()
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

  /** Returns the launcher right speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedRight() {
    return launcherEncoderRight.getVelocity();
  }

  /** Returns the launcher left speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedLeft() {
    return launcherEncoderLeft.getVelocity();
  }

  /** Returns the left launcher motor commanded voltage. */
  public double getLauncherVoltageCommandLeft() {
    return launcherVoltageLeftCommand;
  }

  /** Returns the right launcher motor commanded voltage. */
  public double getLauncherVoltageCommandRight() {
    return launcherVoltageRightCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    launcherLeftController.setP(LauncherConstants.LAUNCHER_KP.getValue());
    launcherRightController.setP(LauncherConstants.LAUNCHER_KP.getValue());

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
