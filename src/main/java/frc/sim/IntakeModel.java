// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.sim.Constants.IntakeSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class IntakeModel implements AutoCloseable {

  private final IntakeSubsystem intakeIntakeSubsystem;
  private double simIntakeCurrent = 0.0;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor intakeGearbox = DCMotor.getAndymarkRs775_125(1);

  private final DCMotorSim intakeMotorSim =
      new DCMotorSim(
          intakeGearbox,
          IntakeConstants.INTAKE_GEAR_RATIO,
          IntakeSimConstants.INTAKE_MOI_KG_METERS2);

  /** Create a new IntakeModel. */
  public IntakeModel(IntakeSubsystem intakeSubsystemToSimulate) {

    intakeIntakeSubsystem = intakeSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Nothing to do
  }

  /** Update the simulation model. */
  public void updateSim() {

    double inputVoltage = intakeIntakeSubsystem.getIntakeVoltageCommand();

    intakeMotorSim.setInput(inputVoltage);

    // Next, we update it. The standard loop time is 20ms.
    intakeMotorSim.update(0.020);

    double simIntakeSpeed = intakeMotorSim.getAngularVelocityRPM();

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    simIntakeCurrent =
        intakeGearbox.getCurrent(1.0, intakeIntakeSubsystem.getIntakeVoltageCommand());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(simIntakeCurrent));

    // Publish speed and current to the dashboard during simulation since this motor setup
    // does not provide data.
    SmartDashboard.putNumber("Intake Sim Speed", simIntakeSpeed);
    SmartDashboard.putNumber("Intake Sim Current", simIntakeCurrent);
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
