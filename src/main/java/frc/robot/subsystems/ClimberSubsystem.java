// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code ClimberSubsystem} class is a subsystem that controls the movement of an climber using
 * a Profiled PID Controller. It uses a CANSparkMax motor and a RelativeEncoder to measure the
 * climber's position. The class provides methods to move the climber to a specific position, hold
 * the climber at the current position, and shift the climber's position up or down by a fixed
 * increment.
 *
 * <p>The ClimberSubsystem class provides a constructor where hardware dependiencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of ClimberSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * climberHardware = new ClimberSubsystem.Hardware(motor, encoder);
 * ClimberSubsystem ClimberSubsystem = new ClimberSubsystem(climberHardware);
 *
 * // Create a new instance of ClimberSubsystem using default hardware
 * ClimberSubsystem ClimberSubsystem = new ClimberSubsystem(initializeHardware());
 *
 * // Move the climber to a specific position
 * Command moveToPositionCommand = ClimberSubsystem.moveToPosition(1.0);
 * moveToPositionCommand.schedule();
 *
 * // Hold the climber at the current position
 * Command holdPositionCommand = ClimberSubsystem.holdPosition();
 * holdPositionCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the movement of an climber using a Profiled PID Controller
 *   - Move the climber to a specific position
 *   - Hold the climber at the current position
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the climber's state.
 *   - {@code useOutput()}: Generates the motor command using the PID controller and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the climber to a new position.
 *   - {@code holdPosition()}: Returns a Command that holds the climber at the last goal position.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the climber has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the climber.
 *   - {@code disable()}: Disables the PID control of the climber.
 *   - {@code getMeasurement()}: Returns the climber position for PID control and logging.
 *   - {@code getVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code initPreferences()}: Initializes the preferences for tuning the controller.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the climber.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the climber's
 *     position.
 *   - {@code private ProfiledPIDController climberController}: The PID controller used to
 *     control the climber's movement.
 *   - {@code private climberFeedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double output}: The output of the PID controller.
 *   - {@code private TrapezoidProfile.State setpoint}: The setpoint of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean climberEnabled}: A flag indicating whether the climber is enabled.
 *   - {@code private double voltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the climber subsystem with left and right motors */
  public static class Hardware {
    CANSparkMax motorLeft;
    CANSparkMax motorRight;
    RelativeEncoder encoderLeft;
    RelativeEncoder encoderRight;

    public Hardware(
        CANSparkMax motorLeft,
        CANSparkMax motorRight,
        RelativeEncoder encoderLeft,
        RelativeEncoder encoderRight) {
      this.motorLeft = motorLeft;
      this.motorRight = motorRight;
      this.encoderLeft = encoderLeft;
      this.encoderRight = encoderRight;
    }
  }

  private final CANSparkMax motorLeft;
  private final CANSparkMax motorRight;
  private final RelativeEncoder encoderLeft;
  private final RelativeEncoder encoderRight;

  private ProfiledPIDController climberController =
      new ProfiledPIDController(
          ClimberConstants.CLIMBER_KP.getValue(),
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              ClimberConstants.CLIMBER_MAX_VELOCITY_METERS_PER_SEC.getValue(),
              ClimberConstants.CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2.getValue()));

  ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ClimberConstants.CLIMBER_KS.getValue(),
          ClimberConstants.CLIMBER_KG.getValue(),
          ClimberConstants.CLIMBER_KV_VOLTS_PER_METER_PER_SEC.getValue(),
          0.0); // Acceleration is not used in this implementation

  private double output = 0.0;
  private TrapezoidProfile.State setpoint = new State();
  private double newFeedforward = 0;
  private boolean climberEnabled;
  private double voltageCommand = 0.0;

  /** Create a new ClimberSubsystem controlled by a Profiled PID COntroller . */
  public ClimberSubsystem(Hardware climberHardware) {
    this.motorLeft = climberHardware.motorLeft;
    this.motorRight = climberHardware.motorRight;
    this.encoderLeft = climberHardware.encoderLeft;
    this.encoderRight = climberHardware.encoderRight;

    initializeClimber();
  }

  private void initializeClimber() {

    RobotPreferences.initPreferencesArray(ClimberConstants.getClimberPreferences());

    initEncoders();
    initMotors();

    // Set tolerances that will be used to determine when the climber is at the goal position.
    climberController.setTolerance(
        ClimberConstants.POSITION_TOLERANCE_METERS, ClimberConstants.VELOCITY_TOLERANCE_METERS);

    disable();
  }

  private void initMotors() {
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    motorLeft.clearFaults();
    motorRight.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    motorLeft.setIdleMode(IdleMode.kBrake);
    motorRight.setIdleMode(IdleMode.kBrake);

    DataLogManager.log("Climber motor left firmware version:" + motorLeft.getFirmwareString());
    DataLogManager.log("Climber motor right firmware version:" + motorRight.getFirmwareString());
  }

  private void initEncoders() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // climber position will only be correct if the climber is in the starting rest position when
    // the subsystem is constructed.
    encoderLeft.setPositionConversionFactor(ClimberConstants.CLIMBER_METERS_PER_ENCODER_ROTATION);
    encoderLeft.setVelocityConversionFactor(ClimberConstants.RPM_TO_METERS_PER_SEC);
    encoderLeft.setPosition(0);

    encoderRight.setPositionConversionFactor(ClimberConstants.CLIMBER_METERS_PER_ENCODER_ROTATION);
    encoderRight.setVelocityConversionFactor(ClimberConstants.RPM_TO_METERS_PER_SEC);
    encoderRight.setPosition(0);
  }

  /**
   * Initialize hardware devices for the climber subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax motorLeft = new CANSparkMax(ClimberConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax motorRight =
        new CANSparkMax(ClimberConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoderLeft = motorLeft.getEncoder();
    RelativeEncoder encoderRight = motorRight.getEncoder();

    return new Hardware(motorLeft, motorRight, encoderLeft, encoderRight);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Climber Enabled", climberEnabled);
    SmartDashboard.putNumber("Climber Goal", climberController.getGoal().position);
    SmartDashboard.putNumber("Climber Left Position", getMeasurementLeft());
    SmartDashboard.putNumber("Climber Right Position", getMeasurementLeft());
    SmartDashboard.putNumber("Climber Left Velocity", encoderLeft.getVelocity());
    SmartDashboard.putNumber("Climber Right Velocity", encoderRight.getVelocity());
    SmartDashboard.putNumber("Climber Left Voltage", voltageCommand);
    SmartDashboard.putNumber("Climber Right Voltage", voltageCommand);
    SmartDashboard.putNumber("Climber Left Current", motorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Climber Right Current", motorRight.getOutputCurrent());
    SmartDashboard.putNumber("Climber Left Feedforward", newFeedforward);
    SmartDashboard.putNumber("Climber Right Feedforward", newFeedforward);
    SmartDashboard.putNumber("Climber Left PID output", output);
    SmartDashboard.putNumber("Climber Right PID output", output);
    SmartDashboard.putNumber("Climber Left SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Climber Right SetPt Vel", setpoint.velocity);
    SmartDashboard.putNumber("Climber Left SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Climber Right SetPt Vel", setpoint.velocity);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void useOutput() {
    if (climberEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      output = climberController.calculate(getMeasurementLeft());
      setpoint = climberController.getSetpoint();

      // Calculate the feedforward to move the climber at the desired velocity and offset
      // the effect of gravity. Voltage for acceleration is not used.
      newFeedforward = feedforward.calculate(setpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;

    } else {
      // If the climber isn't enabled, set the motor command to 0. In this state the climber
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      output = 0;
      newFeedforward = 0;
      voltageCommand = 0;
    }
    motorLeft.setVoltage(voltageCommand);
    motorRight.setVoltage(voltageCommand);
  }

  /** Returns a Command that moves the climber to a new position. */
  public Command moveToPosition(double goal) {
    return new FunctionalCommand(
        () -> setGoalPosition(goal),
        this::useOutput,
        interrupted -> {},
        this::atGoalPosition,
        this);
  }

  /**
   * Returns a Command that holds the climber at the last goal position using the PID Controller
   * driving the motor.
   */
  public Command holdPosition() {
    return run(this::useOutput).withName("Climber: Hold Position");
  }

  /**
   * Set the goal state for the subsystem, limited to allowable range. Goal velocity is set to zero.
   * The ProfiledPIDController drives the climber to this position and holds it there.
   */
  private void setGoalPosition(double goal) {
    climberController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.ClimberConstants.CLIMBER_MIN_HEIGHT_METERS,
                Constants.ClimberConstants.CLIMBER_MAX_HEIGHT_METERS),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the climber has reached the goal position and velocity is within limits. */
  public boolean atGoalPosition() {
    return climberController.atGoal();
  }

  /**
   * Sets up the PID controller to move the climber to the defined goal position and hold at that
   * position. Preferences for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!climberEnabled) {
      loadPreferences();
      setDefaultCommand(holdPosition());

      // Reset the PID controller to clear any previous state
      climberController.reset(getMeasurementLeft());
      climberEnabled = true;

      DataLogManager.log(
          "Climber Enabled - kP="
              + climberController.getP()
              + " kI="
              + climberController.getI()
              + " kD="
              + climberController.getD()
              + " PosGoal="
              + climberController.getGoal().position
              + " CurPos="
              + getMeasurementLeft());
    }
  }

  /**
   * Disables the PID control of the climber. Sets motor output to zero. NOTE: In this state the
   * climber will move until it hits the stop. Using EMF braking mode with motor will slow this
   * movement.
   */
  public void disable() {

    // Clear the enabled flag and call useOutput to zero the motor command
    climberEnabled = false;
    useOutput();

    // Remove the default command and cancel any command that is active
    removeDefaultCommand();
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log(
        "Climber Left Disabled CurPos="
            + getMeasurementLeft()
            + " CurVel="
            + encoderLeft.getVelocity());

    DataLogManager.log(
        "Climber Right Disabled CurPos="
            + getMeasurementRight()
            + " CurVel="
            + encoderRight.getVelocity());
  }

  /**
   * Returns the left climber position for PID control and logging (Units are meters from low
   * position).
   */
  public double getMeasurementLeft() {
    // Add the offset from the starting point. The climber must be at this position at startup for
    // the relative encoder to provide a correct position.
    return encoderLeft.getPosition() + ClimberConstants.CLIMBER_OFFSET_RADS;
  }

  /**
   * Returns the right climber position for PID control and logging (Units are meters from low
   * position).
   */
  public double getMeasurementRight() {
    // Add the offset from the starting point. The climber must be at this position at startup for
    // the relative encoder to provide a correct position.
    return encoderRight.getPosition() + ClimberConstants.CLIMBER_OFFSET_RADS;
  }

  /** Returns the Motor Commanded Voltage. */
  public double getVoltageCommand() {
    return voltageCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    climberController.setP(ClimberConstants.CLIMBER_KP.getValue());

    // Read Preferences for Trapezoid Profile and update
    double velocityMax = ClimberConstants.CLIMBER_MAX_VELOCITY_METERS_PER_SEC.getValue();
    double accelerationMax = ClimberConstants.CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2.getValue();
    climberController.setConstraints(
        new TrapezoidProfile.Constraints(velocityMax, accelerationMax));

    // Read Preferences for Feedforward and create a new instance
    double staticGain = ClimberConstants.CLIMBER_KS.getValue();
    double gravityGain = ClimberConstants.CLIMBER_KG.getValue();
    double velocityGain = ClimberConstants.CLIMBER_KV_VOLTS_PER_METER_PER_SEC.getValue();
    feedforward = new ElevatorFeedforward(staticGain, gravityGain, velocityGain, 0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motorLeft.close();
    motorRight.close();
  }
}
