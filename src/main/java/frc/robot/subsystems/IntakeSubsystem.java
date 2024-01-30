// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The {@code IntakeIntakeSubsystem} class is a subsystem that runs an intake motor and no encoder
 * using an open loop voltage command. It uses a PWMSparkMax motor controller. The class provides
 * methods to return commands to run the intake at the specified voltage or to disable by setting
 * voltage to 0.
 *
 * <p>The IntakeIntakeSubsystem class provides a constructor where hardware dependencies are passed
 * in to allow access for testing. There is also a method provided to create default hardware when
 * those details are not needed outside of the subsystem.
 *
 * <p>Example Usage:
 *
 * <pre>{@code
 * // Create a new instance of IntakeIntakeSubsystem using specified hardware
 * PWMSparkMax motor = new PWMSparkMax(2);
 * intakeHardware = new IntakeIntakeSubsystem.Hardware(motor);
 * IntakeIntakeSubsystem intakeSubsystem = new IntakeIntakeSubsystem(intakeHardware);
 *
 * // Create a new instance of IntakeIntakeSubsystem using default hardware
 * IntakeIntakeSubsystem intakeSubsystem = new IntakeIntakeSubsystem(initializeHardware());
 *
 * // Run the intake at a specific voltage
 * Command runIntakeCommand = intakeSubsystem.runIntake(7.5);
 * runIntakeCommand.schedule();
 *
 * // Stop the intake
 * Command stopIntakeCommand = intakeSubsystem.stopIntake();
 * runIntakeCommand.schedule();
 * }
 *
 * Code Analysis:
 * - Main functionalities:
 *   - Control the speed of an intake using a PID Controller
 * - Methods:
 *   - {@code runIntake(double voltage)}: Returns a Command that runs the intake at the defined
 *      voltage.
 *   - {@code stopIntake()}: Returns a Command that stops the intake by setting voltage to 0.
 *   - {@code getIntakeSpeed()}: Returns the intake position for PID control and logging.
 *   - {@code setIntakeVoltageCommand()}: Set the intake motor commanded voltage and log the new
 *      value.
 *   - {@code getIntakeVoltageCommand()}: Returns the intake motor commanded voltage.
 *   - {@code disableIntake()}: Disable the intake by setting voltage to 0.
 *   - {@code close()}: Closes any objects that support it.
 *   - Fields:
 *   - {@code private final PWMSparkMax motor}: The motor used to control the intake.
 * </pre>
 */
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the intake subsystem. */
  public static class Hardware {
    PWMSparkMax intakeMotor;

    public Hardware(PWMSparkMax motor) {
      this.intakeMotor = motor;
    }
  }

  private final PWMSparkMax motor;

  /** Create a new IntakeSubsystem thats runs a motor with an openloop voltage command. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.motor = intakeHardware.intakeMotor;

    setIntakeVoltageCommand(0);
  }

  /**
   * Create hardware devices for the intake subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    PWMSparkMax intakeMotor = new PWMSparkMax(IntakeConstants.INTAKE_MOTOR_PORT);

    return new Hardware(intakeMotor);
  }

  /** Returns a Command that runs the intake at the defined voltage. */
  public Command runIntake(double voltage) {
    return new FunctionalCommand(
        () -> setIntakeVoltageCommand(voltage), () -> {}, interrupted -> {}, () -> false, this);
  }

  /** Returns a Command that stops the intake by setting voltage to 0. */
  public Command stopIntake() {
    return new FunctionalCommand(
        () -> setIntakeVoltageCommand(0), () -> {}, interrupted -> {}, () -> false, this);
  }

  /** Set the intake motor commanded voltage and log the new value. */
  private void setIntakeVoltageCommand(double voltage) {
    motor.setVoltage(voltage);
    DataLogManager.log("Intake Voltage Set to " + voltage);
    SmartDashboard.putNumber("Intake Voltage", voltage);
  }

  /** Returns the intake motor commanded voltage. */
  public double getIntakeVoltageCommand() {
    return motor.get();
  }

  /** Disable the intake by setting voltage to 0. */
  public void disableIntake() {
    setIntakeVoltageCommand(0);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
