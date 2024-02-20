// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.sim.Constants.LauncherSimConstants;

/**
 * A simulation for a four motor launcher subsystem. Only one motor on each side is modeled and the
 * other is set to the same speed but opposite direction.
 */
public class LauncherModel implements AutoCloseable {

  private final LauncherSubsystem launcherSubsystem;
  private double simLauncherTopCurrent = 0.0;
  private double simLauncherBottomCurrent = 0.0;
  private CANSparkMaxSim sparkTopLeftSim;
  private CANSparkMaxSim sparkTopRightSim;
  private CANSparkMaxSim sparkBottomLeftSim;
  private CANSparkMaxSim sparkBottomRightSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor launcherGearbox = DCMotor.getNEO(1);

  private final DCMotorSim launcherMotorTopSim =
      new DCMotorSim(
          launcherGearbox,
          LauncherConstants.LAUNCHER_GEAR_RATIO,
          LauncherSimConstants.LAUNCHER_MOI_KG_METERS2);

  private final DCMotorSim launcherMotorBottomSim =
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
    sparkTopLeftSim = new CANSparkMaxSim(LauncherConstants.TOP_LEFT_LAUNCHER_MOTOR_PORT);
    sparkTopRightSim = new CANSparkMaxSim(LauncherConstants.TOP_RIGHT_LAUNCHER_MOTOR_PORT);
    sparkBottomLeftSim = new CANSparkMaxSim(LauncherConstants.BOTTOM_LEFT_LAUNCHER_MOTOR_PORT);
    sparkBottomRightSim = new CANSparkMaxSim(LauncherConstants.BOTTOM_RIGHT_LAUNCHER_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {

    launcherMotorTopSim.setInput(launcherSubsystem.getLauncherVoltageCommandTopLeft());
    launcherMotorBottomSim.setInput(launcherSubsystem.getLauncherVoltageCommandBottomLeft());

    // Next, we update it. The standard loop time is 20ms.
    launcherMotorTopSim.update(0.020);
    launcherMotorBottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and save the current so it can be
    // retrieved later.
    sparkTopLeftSim.setVelocity(launcherMotorTopSim.getAngularVelocityRPM());
    sparkTopRightSim.setVelocity(-launcherMotorTopSim.getAngularVelocityRPM());
    sparkTopLeftSim.setPosition(launcherMotorTopSim.getAngularPositionRotations());
    sparkTopRightSim.setPosition(-launcherMotorTopSim.getAngularPositionRotations());
    sparkBottomLeftSim.setVelocity(-launcherMotorBottomSim.getAngularVelocityRPM());
    sparkBottomRightSim.setVelocity(launcherMotorBottomSim.getAngularVelocityRPM());
    sparkBottomLeftSim.setPosition(-launcherMotorBottomSim.getAngularPositionRotations());
    sparkBottomRightSim.setPosition(launcherMotorBottomSim.getAngularPositionRotations());
    simLauncherTopCurrent =
        launcherGearbox.getCurrent(
            launcherMotorTopSim.getAngularVelocityRadPerSec(),
            launcherSubsystem.getLauncherVoltageCommandTopLeft());
    simLauncherBottomCurrent =
        launcherGearbox.getCurrent(
            launcherMotorBottomSim.getAngularVelocityRadPerSec(),
            launcherSubsystem.getLauncherVoltageCommandBottomLeft());
    sparkTopLeftSim.setCurrent(simLauncherTopCurrent);
    sparkTopRightSim.setCurrent(-simLauncherTopCurrent);
    sparkBottomLeftSim.setCurrent(-simLauncherBottomCurrent);
    sparkBottomRightSim.setCurrent(simLauncherBottomCurrent);
  }

  /** Return the left simulated current. */
  public double getSimTopCurrent() {
    return simLauncherTopCurrent;
  }

  /** Return the right simulated current. */
  public double getSimBottomCurrent() {
    return simLauncherBottomCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
