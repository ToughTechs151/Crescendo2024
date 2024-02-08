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
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotPreferences;

/**
 * The {@code IntakeSubsystem} class is a subsystem that controls the speed of an intake using a PID
 * Controller and simple motor feedforward. It uses a CANSparkMax motor and a RelativeEncoder to
 * measure the intake's speed. The class provides methods to return commands that run the intake at
 * the specified speed or stop the motor.
 *
 * <p>The IntakeSubsystem class provides a constructor where hardware dependencies are passed in to
 * allow access for testing. There is also a method provided to create default hardware when those
 * details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of IntakeSubsystem using specified hardware
 * CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
 * RelativeEncoder encoder = motor.getEncoder();
 * intakeHardware = new IntakeSubsystem.Hardware(motor, encoder);
 * IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeHardware);
 *
 * // Create a new instance of IntakeSubsystem using default hardware
 * IntakeSubsystem intakeSubsystem = new IntakeSubsystem(initializeHardware());
 *
 * // Run the intake at a specific speed
 * Command runIntakeCommand = intakeSubsystem.runIntake(500.0);
 * runIntakeCommand.schedule();
 *
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an intake using a PID Controller
 * - Methods:
 *   - {@code periodic()}: Publish telemetry with information about the intake's state.
 *   - {@code updateMotorController()}: Generates the motor command using the PID controller and
 *     feedforward.
 *   - {@code runForward()}: Returns a Command that runs the motor forward at the current set
 *     speed.
 *   - {@code runReverse()}: Returns a Command that runs the motor in reverse at the current set
 *     speed.
 *   - {@code setMotorSetPoint(double scale)}: Set the setpoint for the motor as a scale factor
 *     applied to the setpoint value.
 *   - {@code intakeAtSetpoint()}: Returns whether the intake has reached the set point velocity
 *     within limits.
 *   - {@code enableIntake()}: Enables the PID control of the intake.
 *   - {@code disableIntake()}: Disables the PID control of the intake.
 *   - {@code getIntakeSpeed()}: Returns the intake position for PID control and logging.
 *   - {@code getIntakeVoltageCommand()}: Returns the motor commanded voltage.
 *   - {@code loadPreferences()}: Loads the preferences for tuning the controller.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final CANSparkMax motor}: The motor used to control the intake.
 *   - {@code private final RelativeEncoder encoder}: The encoder used to measure the intake's
 *     position.
 *   - {@code private PIDController intakeController}: The PID controller used to
 *     control the intake's speed.
 *   - {@code private Feedforward feedforward}: The feedforward controller used to
 *     calculate the motor output.
 *   - {@code private double pidOutput}: The output of the PID controller.
 *   - {@code private double newFeedforward}: The calculated feedforward value.
 *   - {@code private boolean intakeEnabled}: A flag indicating whether the intake is enabled.
 *   - {@code private double intakeVoltageCommand}: The motor commanded voltage.
 *   - {@code private double setSpeed}: The setpoint speed for the controller.
 * </pre>
 */
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the intake subsystem. */
  public static class Hardware {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public Hardware(CANSparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;

  private PIDController intakeController =
      new PIDController(IntakeConstants.INTAKE_KP.getValue(), 0.0, 0.0);

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          IntakeConstants.INTAKE_KS_VOLTS.getValue(),
          IntakeConstants.INTAKE_KV_VOLTS_PER_RPM.getValue(),
          IntakeConstants.INTAKE_KA_VOLTS_PER_RPM2.getValue());

  private double pidOutput = 0.0;
  private double newFeedforward = 0;
  private boolean intakeEnabled;
  private double intakeVoltageCommand = 0.0;

  private double setSpeed;

  /** Create a new IntakeSubsystem controlled by a Profiled PID COntroller . */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.intakeMotor = intakeHardware.motor;
    this.intakeEncoder = intakeHardware.encoder;

    initializeIntake();
  }

  private void initializeIntake() {

    RobotPreferences.initPreferencesArray(IntakeConstants.getIntakePreferences());

    initIntakeMotor();
    initIntakeEncoder();

    // Set tolerances that will be used to determine when the intake is at the goal velocity.
    intakeController.setTolerance(IntakeConstants.INTAKE_TOLERANCE_RPM);

    disableIntake();
  }

  private void initIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
    // Maybe we should print the faults if non-zero before clearing?
    intakeMotor.clearFaults();
    // Configure the motor to use EMF braking when idle and set voltage to 0.
    intakeMotor.setIdleMode(IdleMode.kBrake);
    DataLogManager.log("Intake motor firmware version:" + intakeMotor.getFirmwareString());
  }

  private void initIntakeEncoder() {
    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // intake position will only be correct if the intake is in the starting rest position when
    // the subsystem is constructed.
    intakeEncoder.setPositionConversionFactor(
        IntakeConstants.INTAKE_ROTATIONS_PER_ENCODER_ROTATION);
    intakeEncoder.setVelocityConversionFactor(
        IntakeConstants.INTAKE_ROTATIONS_PER_ENCODER_ROTATION);
  }

  /**
   * Create hardware devices for the intake subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax intakeMotor =
        new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    return new Hardware(intakeMotor, intakeEncoder);
  }

  /** Publish telemetry with information about the intake's state. */
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
    SmartDashboard.putNumber("Intake Setpoint", intakeController.getSetpoint());
    SmartDashboard.putNumber("Intake Speed", intakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Voltage", intakeVoltageCommand);
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Feedforward", newFeedforward);
    SmartDashboard.putNumber("Intake PID output", pidOutput);
  }

  /** Generate the motor command using the PID controller output and feedforward. */
  public void updateMotorController() {
    if (intakeEnabled) {
      // Calculate the the motor command by adding the PID controller output and feedforward to run
      // the intake at the desired speed. Store the individual values for logging.
      pidOutput = intakeController.calculate(getIntakeSpeed());
      newFeedforward = feedforward.calculate(intakeController.getSetpoint());
      intakeVoltageCommand = pidOutput + newFeedforward;

    } else {
      // If the intake isn't enabled, set the motor command to 0. In this state the intake
      // will slow down until it stops. Motor EMF braking will cause it to slow down faster
      // if that mode is used.
      pidOutput = 0;
      newFeedforward = 0;
      intakeVoltageCommand = 0;
    }
    intakeMotor.setVoltage(intakeVoltageCommand);
  }

  /** Returns a Command that runs the motor forward at the current set speed. */
  public Command runForward() {
    return new FunctionalCommand(
        () -> setMotorSetPoint(1.0),
        this::updateMotorController,
        interrupted -> disableIntake(),
        () -> false,
        this);
  }

  /** Returns a Command that runs the motor in reverse at the current set speed. */
  public Command runReverse() {
    return new FunctionalCommand(
        () -> setMotorSetPoint(-1.0),
        this::updateMotorController,
        interrupted -> disableIntake(),
        () -> false,
        this);
  }

  /**
   * Set the setpoint for the motor as a scale factor applied to the setpoint value. The
   * PIDController drives the motor to this speed and holds it there.
   */
  private void setMotorSetPoint(double scale) {
    loadPreferences();
    intakeController.setSetpoint(scale * setSpeed);

    // Call enable() to configure and start the controller in case it is not already enabled.
    enableIntake();
  }

  /** Returns whether the intake has reached the set point speed within limits. */
  public boolean intakeAtSetpoint() {
    return intakeController.atSetpoint();
  }

  /**
   * Sets up the PID controller to run the intake at the defined setpoint speed. Preferences for
   * tuning the controller are applied.
   */
  private void enableIntake() {

    // Don't enable if already enabled since this may cause control transients
    if (!intakeEnabled) {
      loadPreferences();

      // Reset the PID controller to clear any previous state
      intakeController.reset();
      intakeEnabled = true;

      DataLogManager.log(
          "Intake Enabled - kP="
              + intakeController.getP()
              + " kI="
              + intakeController.getI()
              + " kD="
              + intakeController.getD()
              + " Setpoint="
              + intakeController.getSetpoint()
              + " CurSpeed="
              + getIntakeSpeed());
    }
  }

  /**
   * Disables the PID control of the intake. Sets motor output to zero. NOTE: In this state the
   * intake will slow down until it stops. Motor EMF braking will cause it to slow down faster if
   * that mode is used.
   */
  public void disableIntake() {

    // Clear the enabled flag and update the controller to zero the motor command
    intakeEnabled = false;
    updateMotorController();

    // Cancel any command that is active
    Command currentCommand = CommandScheduler.getInstance().requiring(this);
    if (currentCommand != null) {
      CommandScheduler.getInstance().cancel(currentCommand);
    }
    DataLogManager.log("Intake Disabled CurSpeed=" + getIntakeSpeed());
  }

  /** Returns the intake speed for PID control and logging (Units are RPM). */
  public double getIntakeSpeed() {
    return intakeEncoder.getVelocity();
  }

  /** Returns the intake motor commanded voltage. */
  public double getIntakeVoltageCommand() {
    return intakeVoltageCommand;
  }

  /**
   * Load Preferences for values that can be tuned at runtime. This should only be called when the
   * controller is disabled - for example from enable().
   */
  private void loadPreferences() {

    // Read the motor speed set point
    setSpeed = IntakeConstants.INTAKE_SET_POINT_RPM.getValue();

    // Read Preferences for PID controller
    intakeController.setP(IntakeConstants.INTAKE_KP.getValue());

    // Read Preferences for Feedforward and create a new instance
    double staticGain = IntakeConstants.INTAKE_KS_VOLTS.getValue();
    double velocityGain = IntakeConstants.INTAKE_KV_VOLTS_PER_RPM.getValue();
    double accelerationGain = IntakeConstants.INTAKE_KA_VOLTS_PER_RPM2.getValue();
    feedforward = new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    intakeMotor.close();
  }
}
