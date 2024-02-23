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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code LauncherSubsystem} class is a subsystem that controls the speed of the launcher using
 * PID Controllers and simple motor feedforward. It uses four CANSparkMax motors with
 * RelativeEncoders to measure the launcher's speed. The class provides methods to return commands
 * that run the launcher motors at the specified speeds or stop the motors. Top and bottom motors
 * have separate set speeds.
 *
 * <p>The LauncherSubsystem class provides a constructor where hardware dependencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of LauncherSubsystem using specified hardware
 * CANSparkMax motorTopRight = new CANSparkMax(1, MotorType.kBrushless);
 * CANSparkMax motorTopLeft = new CANSparkMax(2, MotorType.kBrushless);
 * CANSparkMax motorBottomRight = new CANSparkMax(3, MotorType.kBrushless);
 * CANSparkMax motorBottomLeft = new CANSparkMax(4, MotorType.kBrushless);
 * RelativeEncoder encoderTopRight = motorTopRight.getEncoder();
 * RelativeEncoder encoderTopLeft = motorTopLeft.getEncoder();
 * RelativeEncoder encoderBottomRight = motorBottomRight.getEncoder();
 * RelativeEncoder encoderBottomLeft = motorBottomLeft.getEncoder();
 * launcherHardware = new LauncherSubsystem.Hardware(motorTopRight, motorTopLeft, motorBottomRight,
 * motorBottomLeft, encoderTopRight, encoderTopLeft, encoderBottomRight, encoderBottomLeft);
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(launcherHardware);
 *
 * // Create a new instance of LauncherSubsystem using default hardware
 * LauncherSubsystem launcherSubsystem = new LauncherSubsystem(initializeHardware());
 *
 * // Run the launcher at a specific top and bottom speeds
 * Command runLauncherCommand = launcherSubsystem.runLauncher();
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
 *   - {@code runLauncher()}: Returns a Command that runs the launcher at the
 *     defined top and bottom speeds.
 *   - {@code setSetPoint()}: Set the setpoints for the launcher.
 *   - {@code atSetpoint()}: Returns whether all of the launcher motors have reached the set point
 *     velocity within limits.
 *   - {@code enable()}: Enables the PID control of the launcher.
 *   - {@code disable()}: Disables the PID control of the launcher.
 *   - {@code getLauncherSpeedTopRight()}: Returns the top right speed for PID control and logging.
 *   - {@code getLauncherSpeedTopLeft()}: Returns the top left for PID control and logging.
 *   - {@code getLauncherSpeedBottomRight()}: Returns the bottom right speed for PID control and
 *     logging.
 *   - {@code getLauncherSpeedBottomLeft()}: Returns the bottom left speed for PID control and
 *     logging.
 *   - {@code getLauncherVoltageCommandTopLeft()}: Returns the top left motor commanded voltage.
 *   - {@code getLauncherVoltageCommandTopRight()}: Returns the top right motor commanded voltage.
 *   - {@code getLauncherVoltageCommandBottomLeft()}: Returns the bottom left motor commanded
 *     voltage.
 *   - {@code getLauncherVoltageCommandBottomRight()}: Returns the bottom right motor commanded
 *     voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax launcherMotorTopRight}: The top right motor used to control
 *     the launcher.
 *   - {@code private final CANSparkMax launcherMotorTopLeft}: The top left motor used to control
 *     the launcher.
 *   - {@code private final CANSparkMax launcherMotorBottomRight}: The bottom right  motor used to
 *     control the launcher.
 *   - {@code private final CANSparkMax launcherMotorBottomLeft}: The bottom left  motor used to
 *     control the launcher.
 *   - {@code private final RelativeEncoder launcherEncoderTopRight}: The top right  encoder used to
 *      measure the launcher's speed.
 *   - {@code private final RelativeEncoder launcherEncodeTopLeft}: The top left  encoder used to
 *      measure the launcher's speed.
 *   - {@code private final RelativeEncoder launcherEncoderBottomRight}: The bottom right  encoder
 *     used to measure the launcher's speed.
 *   - {@code private final RelativeEncoder launcherEncodeBottomLeft}: The bottom left  encoder
 *     used to measure the launcher's speed.
 *   - {@code private PIDController launcherTopRightController}: The PID controller used to
 *     control the top right launcher's speed.
 *   - {@code private PIDController launcherTopLeftController}: The PID controller used to
 *     control the top left launcher's speed.
 *   - {@code private PIDController launcherBottomRightController}: The PID controller used to
 *     control the bottom right launcher's speed.
 *   - {@code private PIDController launcherBottomLeftController}: The PID controller used to
 *     control the bottom left launcher's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private SlewRateLimiter topRightLimiter}: Slew limiter to control ramp up of the
 *     top right motor.
 *   - {@code private SlewRateLimiter topLeftLimiter}: Slew limiter to control ramp up of the
 *     top left motor.
 *   - {@code private SlewRateLimiter bottomRightLimiter}: Slew limiter to control ramp up of the
 *     bottom right motor.
 *   - {@code private SlewRateLimiter bottomLeftLimiter}: Slew limiter to control ramp up of the
 *     bottom left motor.
 *   - {@code private double pidTopRightOutput}: The output of the top right PID controller.
 *   - {@code private double pidTopLeftOutput}: The output of the top left PID controller.
 *   - {@code private double pidBottomRightOutput}: The output of the bottom right PID controller.
 *   - {@code private double piBottomLeftOutput}: The output of the bottom left PID controller.
 *   - {@code private double newTopRightFeedforward}: The calculated top right feedforward value.
 *   - {@code private double newTopLeftFeedforward}: The calculated top left feedforward value.
 *   - {@code private double newBottomRightFeedforward}: The calculated bottom right feedforward
 *     value.
 *   - {@code private double newBottomLeftFeedforward}: The calculated bottom left feedforward
 *     value.
 *   - {@code private boolean launcherEnabled}: A flag indicating whether the launcher is enabled.
 *   - {@code private double launcherVoltageTopRightCommand}: The top right motor commanded voltage.
 *   - {@code private double launcherVoltageTopLeftCommand}: The top left motor commanded voltage.
 *   - {@code private double launcherVoltageBottomRightCommand}: The bottom right motor commanded
 *     voltage.
 *   - {@code private double launcherVoltageBottomLeftCommand}: The bottom left motor commanded
 *     voltage.
 * </pre>
 */
public class LauncherSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the launcher subsystem. */
  public static class Hardware {
    CANSparkMax launcherMotorTopRight;
    CANSparkMax launcherMotorTopLeft;
    CANSparkMax launcherMotorBottomRight;
    CANSparkMax launcherMotorBottomLeft;
    RelativeEncoder launcherEncoderTopRight;
    RelativeEncoder launcherEncoderTopLeft;
    RelativeEncoder launcherEncoderBottomRight;
    RelativeEncoder launcherEncoderBottomLeft;

    /** Construct the hardware class to hold the motors and encoders. */
    public Hardware(
        CANSparkMax launcherMotorTopRight,
        CANSparkMax launcherMotorTopLeft,
        CANSparkMax launcherMotorBottomRight,
        CANSparkMax launcherMotorBottomLeft,
        RelativeEncoder launcherEncoderTopRight,
        RelativeEncoder launcherEncoderTopLeft,
        RelativeEncoder launcherEncoderBottomRight,
        RelativeEncoder launcherEncoderBottomLeft) {
      this.launcherMotorTopRight = launcherMotorTopRight;
      this.launcherMotorTopLeft = launcherMotorTopLeft;
      this.launcherMotorBottomLeft = launcherMotorBottomLeft;
      this.launcherMotorBottomRight = launcherMotorBottomRight;
      this.launcherEncoderTopRight = launcherEncoderTopRight;
      this.launcherEncoderTopLeft = launcherEncoderTopLeft;
      this.launcherEncoderBottomRight = launcherEncoderBottomRight;
      this.launcherEncoderBottomLeft = launcherEncoderBottomLeft;
    }
  }

  private final CANSparkMax launcherMotorTopRight;
  private final CANSparkMax launcherMotorTopLeft;
  private final CANSparkMax launcherMotorBottomRight;
  private final CANSparkMax launcherMotorBottomLeft;
  private final RelativeEncoder launcherEncoderTopRight;
  private final RelativeEncoder launcherEncoderTopLeft;
  private final RelativeEncoder launcherEncoderBottomRight;
  private final RelativeEncoder launcherEncoderBottomLeft;

  private PIDController launcherTopRightController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);
  private PIDController launcherTopLeftController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);
  private PIDController launcherBottomRightController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);
  private PIDController launcherBottomLeftController =
      new PIDController(LauncherConstants.LAUNCHER_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          LauncherConstants.LAUNCHER_KS_VOLTS.getValue(),
          LauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue(),
          LauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue());

  SlewRateLimiter topRightLimiter =
      new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
  SlewRateLimiter topLeftLimiter =
      new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
  SlewRateLimiter bottomRightLimiter =
      new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
  SlewRateLimiter bottomLeftLimiter =
      new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());

  private double pidTopRightOutput = 0.0;
  private double pidTopLeftOutput = 0.0;
  private double pidBottomRightOutput = 0.0;
  private double pidBottomLeftOutput = 0.0;
  private double newTopRightFeedforward = 0;
  private double newTopLeftFeedforward = 0;
  private double newBottomRightFeedforward = 0;
  private double newBottomLeftFeedforward = 0;
  private boolean launcherEnabled;
  private double launcherVoltageTopRightCommand = 0.0;
  private double launcherVoltageTopLeftCommand = 0.0;
  private double launcherVoltageBottomRightCommand = 0.0;
  private double launcherVoltageBottomLeftCommand = 0.0;

  /** Create a new LauncherSubsystem controlled by a Profiled PID COntroller . */
  public LauncherSubsystem(Hardware launcherHardware) {
    this.launcherMotorTopRight = launcherHardware.launcherMotorTopRight;
    this.launcherMotorTopLeft = launcherHardware.launcherMotorTopLeft;
    this.launcherMotorBottomRight = launcherHardware.launcherMotorBottomRight;
    this.launcherMotorBottomLeft = launcherHardware.launcherMotorBottomLeft;
    this.launcherEncoderTopRight = launcherHardware.launcherEncoderTopRight;
    this.launcherEncoderTopLeft = launcherHardware.launcherEncoderTopLeft;
    this.launcherEncoderBottomRight = launcherHardware.launcherEncoderBottomRight;
    this.launcherEncoderBottomLeft = launcherHardware.launcherEncoderBottomLeft;

    initializeLauncher();
  }

  private void initializeLauncher() {

    RobotPreferences.initPreferencesArray(LauncherConstants.getLauncherPreferences());

    initLauncherMotor();
    initLauncherEncoder();

    // Set tolerances that will be used to determine when the launcher is at the goal velocity.
    launcherTopRightController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);
    launcherTopLeftController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);
    launcherBottomRightController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);
    launcherBottomLeftController.setTolerance(LauncherConstants.LAUNCHER_TOLERANCE_RPM);

    disableLauncher();
  }

  private void initLauncherMotor() {
    launcherMotorTopRight.restoreFactoryDefaults();
    launcherMotorTopLeft.restoreFactoryDefaults();
    launcherMotorBottomRight.restoreFactoryDefaults();
    launcherMotorBottomLeft.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    launcherMotorTopRight.clearFaults();
    launcherMotorTopLeft.clearFaults();
    launcherMotorBottomRight.clearFaults();
    launcherMotorBottomLeft.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    launcherMotorTopRight.setIdleMode(IdleMode.kBrake);
    launcherMotorTopLeft.setIdleMode(IdleMode.kBrake);
    launcherMotorBottomRight.setIdleMode(IdleMode.kBrake);
    launcherMotorBottomLeft.setIdleMode(IdleMode.kBrake);
    DataLogManager.log(
        "Launcher TR motor firmware version:" + launcherMotorTopRight.getFirmwareString());
    DataLogManager.log(
        "Launcher TL motor firmware version:" + launcherMotorTopLeft.getFirmwareString());
    DataLogManager.log(
        "Launcher BR motor firmware version:" + launcherMotorBottomRight.getFirmwareString());
    DataLogManager.log(
        "Launcher BL motor firmware version:" + launcherMotorBottomLeft.getFirmwareString());
  }

  private void initLauncherEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // launcher position will only be correct if the launcher is in the starting rest position when
    // the subsystem is constructed.
    launcherEncoderTopRight.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderTopLeft.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderBottomRight.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderBottomLeft.setPositionConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderTopRight.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderTopLeft.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderBottomRight.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
    launcherEncoderBottomLeft.setVelocityConversionFactor(
        LauncherConstants.LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION);
  }

  /**
   * Create hardware devices for the launcher subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax launcherMotorTopRight =
        new CANSparkMax(LauncherConstants.TOP_RIGHT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax launcherMotorTopLeft =
        new CANSparkMax(LauncherConstants.TOP_LEFT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax launcherMotorBottomRight =
        new CANSparkMax(LauncherConstants.BOTTOM_RIGHT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax launcherMotorBottomLeft =
        new CANSparkMax(LauncherConstants.BOTTOM_LEFT_LAUNCHER_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder launcherEncoderTopRight = launcherMotorTopRight.getEncoder();
    RelativeEncoder launcherEncoderTopLeft = launcherMotorTopLeft.getEncoder();
    RelativeEncoder launcherEncoderBottomRight = launcherMotorBottomRight.getEncoder();
    RelativeEncoder launcherEncoderBottomLeft = launcherMotorBottomLeft.getEncoder();
    return new Hardware(
        launcherMotorTopRight,
        launcherMotorTopLeft,
        launcherMotorBottomRight,
        launcherMotorBottomLeft,
        launcherEncoderTopRight,
        launcherEncoderTopLeft,
        launcherEncoderBottomRight,
        launcherEncoderBottomLeft);
  }

  /** Publish telemetry with information about the intake's state. */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Launcher Enabled", launcherEnabled);

    SmartDashboard.putNumber("Launcher Top Left Setpoint", launcherTopLeftController.getSetpoint());
    SmartDashboard.putNumber(
        "Launcher Top Right Setpoint", launcherTopRightController.getSetpoint());
    SmartDashboard.putNumber(
        "Launcher Bottom Left Setpoint", launcherBottomLeftController.getSetpoint());
    SmartDashboard.putNumber(
        "Launcher Bottom Right Setpoint", launcherBottomRightController.getSetpoint());

    SmartDashboard.putNumber("Launcher Top Right Speed", launcherEncoderTopRight.getVelocity());
    SmartDashboard.putNumber("Launcher Top Left Speed", launcherEncoderTopLeft.getVelocity());
    SmartDashboard.putNumber(
        "Launcher Bottom Right Speed", launcherEncoderBottomRight.getVelocity());
    SmartDashboard.putNumber("Launcher Bottom Left Speed", launcherEncoderBottomLeft.getVelocity());

    SmartDashboard.putNumber("Launcher Top Left Voltage", launcherVoltageTopLeftCommand);
    SmartDashboard.putNumber("Launcher Top Right Voltage", launcherVoltageTopRightCommand);
    SmartDashboard.putNumber("Launcher Bottom Left Voltage", launcherVoltageBottomLeftCommand);
    SmartDashboard.putNumber("Launcher Bottom Right Voltage", launcherVoltageBottomRightCommand);

    SmartDashboard.putNumber(
        "Launcher Top Right Current", launcherMotorTopRight.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Top Left Current", launcherMotorTopLeft.getOutputCurrent());
    SmartDashboard.putNumber(
        "Launcher Bottom Right Current", launcherMotorBottomRight.getOutputCurrent());
    SmartDashboard.putNumber(
        "Launcher Bottom Left Current", launcherMotorBottomLeft.getOutputCurrent());

    SmartDashboard.putNumber("Launcher Top Left Feedforward", newTopLeftFeedforward);
    SmartDashboard.putNumber("Launcher Top Right Feedforward", newTopRightFeedforward);
    SmartDashboard.putNumber("Launcher Bottom Left Feedforward", newBottomLeftFeedforward);
    SmartDashboard.putNumber("Launcher Bottom Right Feedforward", newBottomRightFeedforward);

    SmartDashboard.putNumber("Launcher Top Left PID output", pidTopLeftOutput);
    SmartDashboard.putNumber("Launcher Top Right PID output", pidTopRightOutput);
    SmartDashboard.putNumber("Launcher Bottom Left PID output", pidBottomLeftOutput);
    SmartDashboard.putNumber("Launcher Bottom Right PID output", pidBottomRightOutput);
    SmartDashboard.putBoolean("Launcher at Setpoint", launcherAtSetpoint());
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateLauncherController() {
    if (launcherEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the launcher at the desired speed. Store the individual values for logging.
      pidTopRightOutput = launcherTopRightController.calculate(getLauncherSpeedTopRight());
      pidTopLeftOutput = launcherTopLeftController.calculate(getLauncherSpeedTopLeft());
      pidBottomRightOutput = launcherBottomRightController.calculate(getLauncherSpeedBottomRight());
      pidBottomLeftOutput = launcherBottomLeftController.calculate(getLauncherSpeedBottomLeft());
      newTopLeftFeedforward = feedforward.calculate(launcherTopLeftController.getSetpoint());
      newBottomLeftFeedforward = feedforward.calculate(launcherBottomLeftController.getSetpoint());
      newTopRightFeedforward = feedforward.calculate(launcherTopRightController.getSetpoint());
      newBottomRightFeedforward =
          feedforward.calculate(launcherBottomRightController.getSetpoint());

      launcherVoltageTopLeftCommand =
          topLeftLimiter.calculate(pidTopLeftOutput + newTopLeftFeedforward);
      launcherVoltageTopRightCommand =
          topRightLimiter.calculate(pidTopRightOutput + newTopRightFeedforward);
      launcherVoltageBottomLeftCommand =
          bottomLeftLimiter.calculate(pidBottomLeftOutput + newBottomLeftFeedforward);
      launcherVoltageBottomRightCommand =
          bottomRightLimiter.calculate(pidBottomRightOutput + newBottomRightFeedforward);

    } else {
      // If the launcher isn't enabled, set the motor command to 0. In this state the launcher
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidTopLeftOutput = 0;
      pidTopRightOutput = 0;
      pidBottomLeftOutput = 0;
      pidBottomRightOutput = 0;
      newTopLeftFeedforward = 0;
      newBottomLeftFeedforward = 0;
      launcherVoltageTopLeftCommand = 0;
      launcherVoltageTopRightCommand = 0;
      launcherVoltageBottomLeftCommand = 0;
      launcherVoltageBottomRightCommand = 0;
    }
    launcherMotorTopRight.setVoltage(launcherVoltageTopRightCommand);
    launcherMotorTopLeft.setVoltage(launcherVoltageTopLeftCommand);
    launcherMotorBottomRight.setVoltage(launcherVoltageBottomRightCommand);
    launcherMotorBottomLeft.setVoltage(launcherVoltageBottomLeftCommand);
  }

  /** Returns a Command that runs the launcher at the defined speed. */
  public Command runLauncher() {
    return new FunctionalCommand(
        this::setLauncherSetPoint,
        this::updateLauncherController,
        interrupted -> disableLauncher(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the launcher. The PIDController drives the launcher to this speed and
   * holds it there.
   */
  private void setLauncherSetPoint() {
    launcherTopLeftController.setSetpoint(LauncherConstants.LAUNCHER_TOP_SPEED);
    launcherTopRightController.setSetpoint(-LauncherConstants.LAUNCHER_TOP_SPEED);
    launcherBottomLeftController.setSetpoint(-LauncherConstants.LAUNCHER_BOTTOM_SPEED);
    launcherBottomRightController.setSetpoint(LauncherConstants.LAUNCHER_BOTTOM_SPEED);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableLauncher();
  }

  /** Returns whether the launcher has reached the set point speed within limits. */
  public boolean launcherAtSetpoint() {
    return (launcherTopLeftController.atSetpoint()
        && launcherTopRightController.atSetpoint()
        && launcherBottomLeftController.atSetpoint()
        && launcherBottomRightController.atSetpoint());
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
      launcherTopLeftController.reset();
      launcherTopRightController.reset();
      launcherBottomLeftController.reset();
      launcherBottomRightController.reset();
      launcherEnabled = true;

      DataLogManager.log(
          "Launcher Enabled - kP="
              + launcherTopLeftController.getP()
              + " kI="
              + launcherTopLeftController.getI()
              + " kD="
              + launcherTopLeftController.getD()
              + " Setpoint="
              + launcherTopLeftController.getSetpoint()
              + " CurSpeedTopRight="
              + getLauncherSpeedTopRight()
              + " CurSpeedTopLeft="
              + getLauncherSpeedTopLeft()
              + " CurSpeedBottomRight="
              + getLauncherSpeedBottomRight()
              + " CurSpeedBottomLeft="
              + getLauncherSpeedBottomLeft());
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
    DataLogManager.log("Launcher Disabled CurSpeedTopRight=" + getLauncherSpeedTopRight());
    DataLogManager.log("Launcher Disabled CurSpeedTopLeft=" + getLauncherSpeedTopLeft());
    DataLogManager.log("Launcher Disabled CurSpeedBottomRight=" + getLauncherSpeedBottomRight());
    DataLogManager.log("Launcher Disabled CurSpeedBottomLeft=" + getLauncherSpeedBottomLeft());
  }

  /** Returns the launcher top right speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedTopRight() {
    return launcherEncoderTopRight.getVelocity();
  }

  /** Returns the launcher top left speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedTopLeft() {
    return launcherEncoderTopLeft.getVelocity();
  }

  /** Returns the launcher bottom right speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedBottomRight() {
    return launcherEncoderBottomRight.getVelocity();
  }

  /** Returns the launcher bottom left speed for PID control and logging (Units are RPM). */
  public double getLauncherSpeedBottomLeft() {
    return launcherEncoderBottomLeft.getVelocity();
  }

  /** Returns the top left launcher motor commanded voltage. */
  public double getLauncherVoltageCommandTopLeft() {
    return launcherVoltageTopLeftCommand;
  }

  /** Returns the top right launcher motor commanded voltage. */
  public double getLauncherVoltageCommandTopRight() {
    return launcherVoltageTopRightCommand;
  }

  /** Returns the bottom left launcher motor commanded voltage. */
  public double getLauncherVoltageCommandBottomLeft() {
    return launcherVoltageBottomLeftCommand;
  }

  /** Returns the bottom right launcher motor commanded voltage. */
  public double getLauncherVoltageCommandBottomRight() {
    return launcherVoltageBottomRightCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    launcherTopLeftController.setP(LauncherConstants.LAUNCHER_KP.getValue());
    launcherTopRightController.setP(LauncherConstants.LAUNCHER_KP.getValue());
    launcherBottomLeftController.setP(LauncherConstants.LAUNCHER_KP.getValue());
    launcherBottomRightController.setP(LauncherConstants.LAUNCHER_KP.getValue());

    // Read Preferences for Feedforward and create a new instance
    double staticGain = LauncherConstants.LAUNCHER_KS_VOLTS.getValue();
    double velocityGain = LauncherConstants.LAUNCHER_KV_VOLTS_PER_RPM.getValue();
    double accelerationGain = LauncherConstants.LAUNCHER_KA_VOLTS_PER_RPM2.getValue();
    feedforward = new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);

    topLeftLimiter = new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
    topRightLimiter = new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
    bottomLeftLimiter =
        new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
    bottomRightLimiter =
        new SlewRateLimiter(LauncherConstants.LAUNCHER_SLEW_VOLTS_PER_SEC.getValue());
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    launcherMotorTopRight.close();
    launcherMotorTopLeft.close();
    launcherMotorBottomRight.close();
    launcherMotorBottomLeft.close();
  }
}
