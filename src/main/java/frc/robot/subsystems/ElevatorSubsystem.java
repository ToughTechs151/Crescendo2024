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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code ElevatorSubsystem} class is a subsystem that controls the movement of an elevator
 * using a Profiled PID Controller. It uses a CANSparkMax motor and a RelativeEncoder to measure the
 * elevator's position. The class provides methods to move the elevator to a specific position, hold
 * the elevator at the current position, and shift the elevator's position up or down by a fixed
 * increment.
 *
 * <p>The ElevatorSubsystem class provides a constructor where hardware dependiencies are passed in
 * to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of ElevatorSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * elevatorHardware = new ElevatorSubsystem.Hardware(motor, encoder);
 * ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorHardware);
 *
 * // Create a new instance of ElevatorSubsystem using default hardware
 * ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(initializeHardware());
 *
 * // Move the elevator to a specific position
 * Command moveToPositionCommand = elevatorSubsystem.moveToPosition(1.0);
 * moveToPositionCommand.schedule();
 *
 * // Hold the elevator at the current position
 * Command holdPositionCommand = elevatorSubsystem.holdPosition();
 * holdPositionCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the movement of an elevator using a Profiled PID Controller
 *   - Move the elevator to a specific position
 *   - Hold the elevator at the current position
 * - Methods:
 *   - {@code periodic()}: Updates the SmartDashboard with information about the elevator's state.
 *   - {@code useOutput()}: Generates the motor command using the PID controller and feedforward.
 *   - {@code moveToPosition(double goal)}: Returns a Command that moves the elevator to a new position.
 *   - {@code holdPosition()}: Returns a Command that holds the elevator at the last goal position.
 *   - {@code setGoalPosition(double goal)}: Sets the goal state for the subsystem.
 *   - {@code atGoalPosition()}: Returns whether the elevator has reached the goal position.
 *   - {@code enable()}: Enables the PID control of the elevator.
 *   - {@code disable()}: Disables the PID control of the elevator.
 *   - {@code getMeasurement()}: Returns the elevator position for PID control and logging.
 *   - {@code getVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code initPreferences()}: Initializes the preferences for tuning the controller.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the elevator.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the elevator's
 *     position.
 *   - {@code private ProfiledPIDController elevatorController}: The PID controller used to
 *     control the elevator's movement.
 *   - {@code private elevatorFeedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double output}: The output of the PID controller.
 *   - {@code private TrapezoidProfile.State setpoint}: The setpoint of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean elevatorEnabled}: A flag indicating whether the elevator is enabled.
 *   - {@code private double voltageCommand}: The motor commanded voltage.
 * </pre>
 */
public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the elevator subsystem. */
  public static class Hardware {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public Hardware(CANSparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          ElevatorConstants.ELEVATOR_KP.getValue(),
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.ELEVATOR_MAX_VELOCITY_METERS_PER_SEC.getValue(),
              ElevatorConstants.ELEVATOR_MAX_ACCELERATION_METERS_PER_SEC2.getValue()));

  ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.ELEVATOR_KS.getValue(),
          ElevatorConstants.ELEVATOR_KG.getValue(),
          ElevatorConstants.ELEVATOR_KV_VOLTS_PER_METER_PER_SEC.getValue(),
          0.0); // Acceleration is not used in this implementation

  private double output = 0.0;
  private TrapezoidProfile.State setpoint = new State();
  private double newFeedforward = 0;
  private boolean elevatorEnabled;
  private double voltageCommand = 0.0;

  /** Create a new ElevatorSubsystem controlled by a Profiled PID COntroller . */
  public ElevatorSubsystem(Hardware elevatorHardware) {
    this.motor = elevatorHardware.motor;
    this.encoder = elevatorHardware.encoder;

    initializeElevator();
  }

  private void initializeElevator() {

    RobotPreferences.initPreferencesArray(ElevatorConstants.getElevatorPreferences());

    initEncoder();
    initMotor();

    // Set tolerances that will be used to determine when the elevator is at the goal position.
    elevatorController.setTolerance(
        ElevatorConstants.POSITION_TOLERANCE_METERS, ElevatorConstants.VELOCITY_TOLERANCE_METERS);

    disable();
  }

  private void initMotor() {
    motor.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    motor.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    motor.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Elevator motor firmware version:" + motor.getFirmwareString());
  }

  private void initEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // elevator position will only be correct if the elevator is in the starting rest position when
    // the subsystem is constructed.
    encoder.setPositionConversionFactor(ElevatorConstants.ELEVATOR_METERS_PER_ENCODER_ROTATION);
    encoder.setVelocityConversionFactor(ElevatorConstants.RPM_TO_METERS_PER_SEC);
    encoder.setPosition(0);
  }

  /**
   * Initialize hardware devices for the elevator subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax motor = new CANSparkMax(ElevatorConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    return new Hardware(motor, encoder);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Elevator Enabled", elevatorEnabled);
    SmartDashboard.putNumber("Elevator Goal", elevatorController.getGoal().position);
    SmartDashboard.putNumber("Elevator Position", getMeasurement());
    SmartDashboard.putNumber("Elevator Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Elevator Voltage", voltageCommand);
    SmartDashboard.putNumber("Elevator Current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Feedforward", newFeedforward);
    SmartDashboard.putNumber("Elevator PID output", output);
    SmartDashboard.putNumber("Elevator SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Elevator SetPt Vel", setpoint.velocity);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void useOutput() {
    if (elevatorEnabled) {
      // Calculate the next set point along the profile to the goal and the next PID output based
      // on the set point and current position.
      output = elevatorController.calculate(getMeasurement());
      setpoint = elevatorController.getSetpoint();

      // Calculate the feedforward to move the elevator at the desired velocity and offset
      // the effect of gravity. Voltage for acceleration is not used.
      newFeedforward = feedforward.calculate(setpoint.velocity);

      // Add the feedforward to the PID output to get the motor output
      voltageCommand = output + newFeedforward;

    } else {
      // If the elevator isn't enabled, set the motor command to 0. In this state the elevator
      // will move down until it hits the rest position. Motor EMF braking will slow movement
      // if that mode is used.
      output = 0;
      newFeedforward = 0;
      voltageCommand = 0;
    }
    motor.setVoltage(voltageCommand);
  }

  /** Returns a Command that moves the elevator to a new position. */
  public Command moveToPosition(double goal) {
    return new FunctionalCommand(
        () -> setGoalPosition(goal),
        this::useOutput,
        interrupted -> {},
        this::atGoalPosition,
        this);
  }

  /**
   * Returns a Command that holds the elevator at the last goal position using the PID Controller
   * driving the motor.
   */
  public Command holdPosition() {
    return run(this::useOutput).withName("Elevator: Hold Position");
  }

  /**
   * Set the goal state for the subsystem, limited to allowable range. Goal velocity is set to zero.
   * The ProfiledPIDController drives the elevator to this position and holds it there.
   */
  private void setGoalPosition(double goal) {
    elevatorController.setGoal(
        new TrapezoidProfile.State(
            MathUtil.clamp(
                goal,
                Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS,
                Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS),
            0));

    // Call enable() to configure and start the controller in case it is not already enabled.
    enable();
  }

  /** Returns whether the elevator has reached the goal position and velocity is within limits. */
  public boolean atGoalPosition() {
    return elevatorController.atGoal();
  }

  /**
   * Sets up the PID controller to move the elevator to the defined goal position and hold at that
   * position. Preferences for tuning the controller are applied.
   */
  private void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!elevatorEnabled) {
      loadPreferences();
      setDefaultCommand(holdPosition());

      // Reset the PID controller to clear any previous state
      elevatorController.reset(getMeasurement());
      elevatorEnabled = true;

      DataLogManager.log(
          "Elevator Enabled - kP="
              + elevatorController.getP()
              + " kI="
              + elevatorController.getI()
              + " kD="
              + elevatorController.getD()
              + " PosGoal="
              + elevatorController.getGoal().position
              + " CurPos="
              + getMeasurement());
    }
  }

  /**
   * Disables the PID control of the elevator. Sets motor output to zero. NOTE: In this state the
   * elevator will move until it hits the stop. Using EMF braking mode with motor will slow this
   * movement.
   */
  public void disable() {

    // Clear the enabled flag and call useOutput to zero the motor command
    elevatorEnabled = false;
    useOutput();

    // Remove the default command and cancel any command that is active
    removeDefaultCommand();
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log(
        "Elevator Disabled CurPos=" + getMeasurement() + " CurVel=" + encoder.getVelocity());
  }

  /**
   * Returns the elevator position for PID control and logging (Units are meters from low position).
   */
  public double getMeasurement() {
    // Add the offset from the starting point. The elevator must be at this position at startup for
    // the relative encoder to provide a correct position.
    return encoder.getPosition() + ElevatorConstants.ELEVATOR_OFFSET_RADS;
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
    elevatorController.setP(ElevatorConstants.ELEVATOR_KP.getValue());

    // Read Preferences for Trapezoid Profile and update
    double velocityMax = ElevatorConstants.ELEVATOR_MAX_VELOCITY_METERS_PER_SEC.getValue();
    double accelerationMax = ElevatorConstants.ELEVATOR_MAX_ACCELERATION_METERS_PER_SEC2.getValue();
    elevatorController.setConstraints(
        new TrapezoidProfile.Constraints(velocityMax, accelerationMax));

    // Read Preferences for Feedforward and create a new instance
    double staticGain = ElevatorConstants.ELEVATOR_KS.getValue();
    double gravityGain = ElevatorConstants.ELEVATOR_KG.getValue();
    double velocityGain = ElevatorConstants.ELEVATOR_KV_VOLTS_PER_METER_PER_SEC.getValue();
    feedforward = new ElevatorFeedforward(staticGain, gravityGain, velocityGain, 0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
