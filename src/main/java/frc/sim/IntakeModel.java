// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.sim.Constants.IntakeSimConstants;

/** A simulation for a simple DC motor with a load. */
public class IntakeModel implements AutoCloseable {

  private final IntakeSubsystem intakeIntakeSubsystem;
  private double simIntakeCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor intakeGearbox = DCMotor.getNEO(1);

  private final DCMotorSim intakeMotorSim =
      new DCMotorSim(
          intakeGearbox,
          IntakeConstants.INTAKE_GEAR_RATIO,
          IntakeSimConstants.INTAKE_MOI_KG_METERS2);

  /** Create a new ElevatorModel. */
  public IntakeModel(IntakeSubsystem intakeIntakeSubsystemToSimulate) {

    intakeIntakeSubsystem = intakeIntakeSubsystemToSimulate;
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

    double inputVoltage = intakeIntakeSubsystem.getIntakeVoltageCommand();

    intakeMotorSim.setInput(inputVoltage);

    // Next, we update it. The standard loop time is 20ms.
    intakeMotorSim.update(0.020);

    double newPosition = intakeMotorSim.getAngularPositionRotations();
    double simIntakeSpeed = intakeMotorSim.getAngularVelocityRPM();

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.setVelocity(simIntakeSpeed);
    sparkSim.setPosition(newPosition);
    simIntakeCurrent =
        intakeGearbox.getCurrent(1.0, intakeIntakeSubsystem.getIntakeVoltageCommand());
    sparkSim.setCurrent(simIntakeCurrent);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simIntakeCurrent));
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
