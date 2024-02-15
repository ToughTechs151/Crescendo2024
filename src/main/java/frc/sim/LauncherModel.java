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
  private double simLauncherLeftCurrent = 0.0;
  private double simLauncherRightCurrent = 0.0;
  private CANSparkMaxSim sparkLeftSim;
  private CANSparkMaxSim sparkRightSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor launcherGearbox = DCMotor.getNEO(1);

  private final DCMotorSim launcherMotorLeftSim =
      new DCMotorSim(
          launcherGearbox,
          LauncherConstants.LAUNCHER_GEAR_RATIO,
          LauncherSimConstants.LAUNCHER_MOI_KG_METERS2);

  private final DCMotorSim launcherMotorRightSim =
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
    sparkLeftSim = new CANSparkMaxSim(LauncherConstants.LEFT_LAUNCHER_MOTOR_PORT);
    sparkRightSim = new CANSparkMaxSim(LauncherConstants.RIGHT_LAUNCHER_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {

    launcherMotorLeftSim.setInput(launcherSubsystem.getLauncherVoltageCommandLeft());
    launcherMotorRightSim.setInput(launcherSubsystem.getLauncherVoltageCommandRight());

    // Next, we update it. The standard loop time is 20ms.
    launcherMotorLeftSim.update(0.020);
    launcherMotorRightSim.update(0.020);

    // Finally, we set our simulated encoder's readings and save the current so it can be
    // retrieved later.
    sparkLeftSim.setVelocity(launcherMotorLeftSim.getAngularVelocityRPM());
    sparkRightSim.setVelocity(launcherMotorRightSim.getAngularVelocityRPM());
    sparkLeftSim.setPosition(launcherMotorLeftSim.getAngularPositionRotations());
    sparkRightSim.setPosition(launcherMotorRightSim.getAngularPositionRotations());
    simLauncherLeftCurrent =
        launcherGearbox.getCurrent(
            launcherMotorLeftSim.getAngularVelocityRadPerSec(),
            launcherSubsystem.getLauncherVoltageCommandLeft());
    simLauncherRightCurrent =
        launcherGearbox.getCurrent(
            launcherMotorRightSim.getAngularVelocityRadPerSec(),
            launcherSubsystem.getLauncherVoltageCommandRight());
    sparkLeftSim.setCurrent(simLauncherLeftCurrent);
    sparkRightSim.setCurrent(simLauncherRightCurrent);
  }

  /** Return the left simulated current. */
  public double getSimLeftCurrent() {
    return simLauncherLeftCurrent;
  }

  /** Return the right simulated current. */
  public double getSimRightCurrent() {
    return simLauncherRightCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
