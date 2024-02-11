// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.sim.Constants.IntakeSimConstants;

/** A simulation for a simple DC motor with a load. */
public class IntakeModel implements AutoCloseable {

  private final IntakeSubsystem intakeSubsystem;
  private double simIntakeCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The intake driven by one motor.
  private final DCMotor motors = DCMotor.getNEO(1);

  private final FlywheelSim intakeMotorSim =
      new FlywheelSim(
          motors, IntakeConstants.INTAKE_GEAR_RATIO, IntakeSimConstants.INTAKE_MOI_KG_METERS2);

  /** Create a new ElevatorModel. */
  public IntakeModel(IntakeSubsystem intakeSubsystemToSimulate) {

    intakeSubsystem = intakeSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(IntakeConstants.INTAKE_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {

    double inputVoltage = intakeSubsystem.getIntakeVoltageCommand();

    intakeMotorSim.setInput(inputVoltage);

    // Next, we update it. The standard loop time is 20ms.
    intakeMotorSim.update(0.020);

    double newPosition = 0.0;

    // Finally, we set our simulated encoder's readings and save the current so it can be
    // retrieved later.
    sparkSim.setVelocity(intakeMotorSim.getAngularVelocityRPM());
    sparkSim.setPosition(newPosition);
    simIntakeCurrent =
        motors.getCurrent(
            intakeMotorSim.getAngularVelocityRadPerSec(),
            intakeSubsystem.getIntakeVoltageCommand());
    sparkSim.setCurrent(simIntakeCurrent);
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simIntakeCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
