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
 * The {@code ClimberSubsystem} class is a subsystem that controls the movement of a dual climber
 * mechanism using Profiled PID Controllers. It uses CANSparkMax motors and RelativeEncoders to
 * measure the climber's position. The class provides methods to move the climber to a specific
 * position, and hold the climber at the current position.
 *
 * <p>The ClimberSubsystem class provides a constructor where hardware dependencies are passed in to
 * allow access for testing. There is also a method provided to create default hardware when those
 * details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of ClimberSubsystem using specified hardware
 * CANSparkMax motorLeft = new CANSparkMax(1, MotorType.kBrushless);
 * CANSparkMax motorRight = new CANSparkMax(2, MotorType.kBrushless);
 * RelativeEncoder encoderLeft = motorLeft.getEncoder();
 * RelativeEncoder encoderRight = motorRight.getEncoder();
 * climberHardware = new ClimberSubsystem.Hardware(motorLeft, motorRight, encoderLeft,
 *   encoderRight);
 * ClimberSubsystem climber = new ClimberSubsystem(climberHardware);
 *
 * // Create a new instance of ClimberSubsystem using default hardware
 * ClimberSubsystem climber = new ClimberSubsystem(initializeHardware());
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
 *   - Control the movement of a dual climber using Profiled PID Controllers
 *   - Move the climber to a specific position
 *   - Hold the climber at the current position
 * - Methods:
 *   - {@code initializeHardware()}: Initialize hardware devices for the climber subsystem.
 *   - {@code periodic()}: Publish telemetry with information about the climber's state.
 *   - {@code useOutput()}: Generates the motor commands using the PID controllers and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the climber to a new
 *     position.
 *   - {@code holdPosition()}: Returns a Command that holds the climber at the last goal position.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the climber has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the climber.
 *   - {@code disable()}: Disables the PID control of the climber.
 *   - {@code getMeasurementLeft()}: Returns the left climber position for PID control and logging.
 *   - {@code getMeasurementRight()}: Returns the right climber position for PID control and
 *     logging.
 *   - {@code getLeftVoltageCommand()}: Returns the left motor commanded voltage.
 *   - {@code getRightVoltageCommand()}: Returns the right motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motorLeft}: The motor used to control the left climber.
 *   - {@code private final CANSparkMax motorRight}: The motor used to control the right climber.
 *   - {@code private final RelativeEncoder encoderLeft}: The encoder used to measure the climber's
 *     left position.
 *   - {@code private final RelativeEncoder encoderRight}: The encoder used to measure the climber's
 *     right position.
 *   - {@code private ProfiledPIDController climberLeftController}: The PID controller used to
 *     control the left climber's movement.
 *   - {@code private ProfiledPIDController climberRightController}: The PID controller used to
 *     control the right climber's movement.
 *   - {@code private climberFeedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double leftPidOutput}: The output of the left PID controller.
 *   - {@code private double rightPidOutput}: The output of the right PID controller.
 *   - {@code private TrapezoidProfile.State leftSetpoint}: The setpoint of the left PID controller.
 *   - {@code private TrapezoidProfile.State rightSetpoint}: The setpoint of the right PID
 *     controller.
 *   - {@code private double leftFeedforward}: The calculated left feedforward value.
 *   - {@code private double rightFeedforward}: The calculated right feedforward value.
 *   - {@code private boolean climberEnabled}: A flag indicating whether the climber is enabled.
 *   - {@code private double leftVoltageCommand}: The left motor commanded voltage.
 *   - {@code private double rightVoltageCommand}: The right motor commanded voltage.
 * </pre>
 */
public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the climber subsystem with left and right motors. */
  public static class Hardware {
    CANSparkMax motorLeft;
    CANSparkMax motorRight;
    RelativeEncoder encoderLeft;
    RelativeEncoder encoderRight;

    /** Save the hardware components when the object is constructed. */
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

  private ProfiledPIDController climberLeftController =
      new ProfiledPIDController(
          ClimberConstants.CLIMBER_KP.getValue(),
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              ClimberConstants.CLIMBER_MAX_VELOCITY_METERS_PER_SEC.getValue(),
              ClimberConstants.CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2.getValue()));

  private ProfiledPIDController climberRightController =
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

  private double leftPidOutput = 0.0;
  private double rightPidOutput = 0.0;
  private TrapezoidProfile.State leftSetpoint = new State();
  private TrapezoidProfile.State rightSetpoint = new State();
  private double leftFeedforward = 0;
  private double rightFeedforward = 0;
  private boolean climberEnabled;
  private double leftVoltageCommand = 0.0;
  private double rightVoltageCommand = 0.0;

  /** Create a new ClimberSubsystem controlled by Profiled PID COntrollers. */
  public ClimberSubsystem(Hardware climberHardware) {
    this.motorLeft = climberHardware.motorLeft;
    this.motorRight = climberHardware.motorRight;
    this.encoderLeft = climberHardware.encoderLeft;
    this.encoderRight = climberHardware.encoderRight;

    initializeClimber();
  }

  private void initializeClimber() {

    RobotPreferences.initPreferencesArray(ClimberConstants.getClimberPreferences());

    initMotors();
    initEncoders();

    // Set tolerances that will be used to determine when the climber is at the goal position.
    climberLeftController.setTolerance(
        ClimberConstants.POSITION_TOLERANCE_METERS, ClimberConstants.VELOCITY_TOLERANCE_METERS);
    climberRightController.setTolerance(
        ClimberConstants.POSITION_TOLERANCE_METERS, ClimberConstants.VELOCITY_TOLERANCE_METERS);

    disable();
  }

  private void initMotors() {
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    motorLeft.clearFaults();
    motorRight.clearFaults();
    // Configure the motor to use EMF braking when idle.
    setBrakeMode(true);

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
    SmartDashboard.putNumber("Climber Goal", climberLeftController.getGoal().position);
    SmartDashboard.putNumber("Climber Left Position", getMeasurementLeft());
    SmartDashboard.putNumber("Climber Right Position", getMeasurementRight());
    SmartDashboard.putNumber("Climber Left Velocity", encoderLeft.getVelocity());
    SmartDashboard.putNumber("Climber Right Velocity", encoderRight.getVelocity());
    SmartDashboard.putNumber("Climber Left Voltage", leftVoltageCommand);
    SmartDashboard.putNumber("Climber Right Voltage", leftVoltageCommand);
    SmartDashboard.putNumber("Climber Left Current", motorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Climber Right Current", motorRight.getOutputCurrent());
    SmartDashboard.putNumber("Climber Left Temp", motorLeft.getMotorTemperature());
    SmartDashboard.putNumber("Climber Right Temp", motorRight.getMotorTemperature());
    SmartDashboard.putNumber("Climber Left Feedforward", leftFeedforward);
    SmartDashboard.putNumber("Climber Right Feedforward", rightFeedforward);
    SmartDashboard.putNumber("Climber Left PID output", leftPidOutput);
    SmartDashboard.putNumber("Climber Right PID output", rightPidOutput);
    SmartDashboard.putNumber("Climber Left SetPt Pos", leftSetpoint.position);
    SmartDashboard.putNumber("Climber Left SetPt Vel", leftSetpoint.velocity);
    SmartDashboard.putNumber("Climber Right SetPt Pos", rightSetpoint.position);
    SmartDashboard.putNumber("Climber Right SetPt Vel", rightSetpoint.velocity);
  }

  /** Generate the motor commands using the PID controller outputs and feedforward. */
  public void useOutput() {
    if (climberEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      leftPidOutput = climberLeftController.calculate(getMeasurementLeft());
      rightPidOutput = climberRightController.calculate(getMeasurementRight());
      leftSetpoint = climberLeftController.getSetpoint();
      rightSetpoint = climberRightController.getSetpoint();

      // Calculate the feedforward to move the climber at the desired velocity and offset
      // the effect of gravity. Voltage for acceleration is not used.
      leftFeedforward = feedforward.calculate(leftSetpoint.velocity);
      rightFeedforward = feedforward.calculate(rightSetpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      leftVoltageCommand = leftPidOutput + leftFeedforward;
      rightVoltageCommand = rightPidOutput + rightFeedforward;

    } else {
      // If the climber isn't enabled, set the motor command to 0. In this state the climber
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      leftPidOutput = 0;
      rightPidOutput = 0;
      leftFeedforward = 0;
      rightFeedforward = 0;
      leftVoltageCommand = 0;
      rightVoltageCommand = 0;
    }
    motorLeft.setVoltage(leftVoltageCommand);
    motorRight.setVoltage(rightVoltageCommand);
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
    climberLeftController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.ClimberConstants.CLIMBER_MIN_PULL_METERS,
                Constants.ClimberConstants.CLIMBER_MAX_PULL_METERS),
            0));

    climberRightController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.ClimberConstants.CLIMBER_MIN_PULL_METERS,
                Constants.ClimberConstants.CLIMBER_MAX_PULL_METERS),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the climber has reached the goal position and velocity is within limits. */
  public boolean atGoalPosition() {
    return (climberLeftController.atGoal() && climberRightController.atGoal());
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
      climberLeftController.reset(getMeasurementLeft());
      climberRightController.reset(getMeasurementRight());
      climberEnabled = true;

      DataLogManager.log(
          "Climber Enabled - kP="
              + climberLeftController.getP()
              + " kI="
              + climberLeftController.getI()
              + " kD="
              + climberLeftController.getD()
              + " PosGoal="
              + climberLeftController.getGoal().position
              + " LeftPos="
              + getMeasurementLeft()
              + " RightPos="
              + getMeasurementRight());
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
        "Climber Disabled: LeftPos="
            + getMeasurementLeft()
            + " LeftVel="
            + encoderLeft.getVelocity()
            + " RightPos="
            + getMeasurementRight()
            + " RightVel="
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

  /** Returns the Left Motor Commanded Voltage. */
  public double getLeftVoltageCommand() {
    return leftVoltageCommand;
  }

  /** Returns the Right Motor Commanded Voltage. */
  public double getRightVoltageCommand() {
    return rightVoltageCommand;
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    if (enableBrake) {
      DataLogManager.log("Climber motors set to brake mode");
      this.motorLeft.setIdleMode(IdleMode.kBrake);
      this.motorRight.setIdleMode(IdleMode.kBrake);
    } else {
      DataLogManager.log("Climber motors set to coast mode");
      this.motorLeft.setIdleMode(IdleMode.kCoast);
      this.motorRight.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Reset the encoders to the start (zero) position. This should only be done when the climber is
   * extended. This command doesn't work in simulation.
   */
  public void resetEncoders() {
    DataLogManager.log("Climber encoders reset");
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read Preferences for PID controller
    climberLeftController.setP(ClimberConstants.CLIMBER_KP.getValue());
    climberRightController.setP(ClimberConstants.CLIMBER_KP.getValue());

    // Read Preferences for Trapezoid Profile and update
    double velocityMax = ClimberConstants.CLIMBER_MAX_VELOCITY_METERS_PER_SEC.getValue();
    double accelerationMax = ClimberConstants.CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2.getValue();
    climberLeftController.setConstraints(
        new TrapezoidProfile.Constraints(velocityMax, accelerationMax));
    climberRightController.setConstraints(
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
