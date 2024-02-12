// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.sim.Constants.LauncherSimConstants;

/** A simulation for a simple DC motor with a load. */
public class LauncherModel implements AutoCloseable {

  private final LauncherSubsystem launcherSubsystem;
  private double simLauncherCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor launcherGearbox = DCMotor.getNEO(1);

  private final DCMotorSim launcherMotorSim =
      new DCMotorSim(
          launcherGearbox,
          LauncherConstants.LAUNCHER_GEAR_RATIO,
          LauncherSimConstants.LAUNCHER_MOI_KG_METERS2);

  /** Create a new ElevatorModel. */
  public LauncherModel(LauncherSubsystem launcherSubsystemToSimulate) {

    launcherSubsystem = launcherSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(LauncherConstants.RIGHT_LAUNCHER_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {

    double inputVoltage = launcherSubsystem.getLauncherVoltageCommand();

    launcherMotorSim.setInput(inputVoltage);

    // Next, we update it. The standard loop time is 20ms.
    launcherMotorSim.update(0.020);

    double newPosition = launcherMotorSim.getAngularPositionRotations();
    double simLauncherSpeed = launcherMotorSim.getAngularVelocityRPM();

    // Finally, we set our simulated encoder's readings and save the current so it can be
    // retrieved later.
    sparkSim.setVelocity(simLauncherSpeed);
    sparkSim.setPosition(newPosition);
    simLauncherCurrent =
        launcherGearbox.getCurrent(
            launcherMotorSim.getAngularVelocityRadPerSec(),
            launcherSubsystem.getLauncherVoltageCommand());
    sparkSim.setCurrent(simLauncherCurrent);
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simLauncherCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
