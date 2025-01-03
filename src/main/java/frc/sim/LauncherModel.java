// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
  private SparkMaxSim sparkTopLeftSim;
  private SparkMaxSim sparkTopRightSim;
  private SparkMaxSim sparkBottomLeftSim;
  private SparkMaxSim sparkBottomRightSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor launcherGearbox = DCMotor.getNEO(1);

  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createDCMotorSystem(
          launcherGearbox,
          LauncherSimConstants.LAUNCHER_MOI_KG_METERS2,
          LauncherConstants.LAUNCHER_GEAR_RATIO);

  private final DCMotorSim launcherMotorTopSim = new DCMotorSim(plant, launcherGearbox);
  private final DCMotorSim launcherMotorBottomSim = new DCMotorSim(plant, launcherGearbox);

  /** Create a new ElevatorModel. */
  public LauncherModel(LauncherSubsystem launcherSubsystemToSimulate) {

    launcherSubsystem = launcherSubsystemToSimulate;
    simulationInit();

    // There is nothing to add to the dashboard for this sim since output is motor speed.
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkTopLeftSim = new SparkMaxSim(launcherSubsystem.getTopLeftMotor(), launcherGearbox);
    sparkTopRightSim = new SparkMaxSim(launcherSubsystem.getTopRightMotor(), launcherGearbox);
    sparkBottomLeftSim = new SparkMaxSim(launcherSubsystem.getBottomLeftMotor(), launcherGearbox);
    sparkBottomRightSim = new SparkMaxSim(launcherSubsystem.getBottomRightMotor(), launcherGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {

    launcherMotorTopSim.setInput(launcherSubsystem.getLauncherVoltageCommandTopLeft());
    launcherMotorBottomSim.setInput(launcherSubsystem.getLauncherVoltageCommandBottomLeft());

    // Next, we update it. The standard loop time is 20ms.
    launcherMotorTopSim.update(0.020);
    launcherMotorBottomSim.update(0.020);

    // Finally, we run the spark simulation and save the current so it can be
    // retrieved later. Left side is modelled and right side is set to inverse of left.
    sparkTopLeftSim.iterate(launcherMotorTopSim.getAngularVelocityRPM(), 12.0, 0.02);
    sparkTopRightSim.iterate(-launcherMotorTopSim.getAngularVelocityRPM(), 12.0, 0.02);
    sparkBottomLeftSim.iterate(launcherMotorTopSim.getAngularVelocityRPM(), 12.0, 0.02);
    sparkBottomRightSim.iterate(-launcherMotorTopSim.getAngularVelocityRPM(), 12.0, 0.02);

    simLauncherTopCurrent =
        Math.abs(
            launcherGearbox.getCurrent(
                launcherMotorTopSim.getAngularVelocityRadPerSec(),
                launcherSubsystem.getLauncherVoltageCommandTopLeft()));
    simLauncherBottomCurrent =
        Math.abs(
            launcherGearbox.getCurrent(
                launcherMotorBottomSim.getAngularVelocityRadPerSec(),
                launcherSubsystem.getLauncherVoltageCommandBottomLeft()));
  }

  /** Return the top simulated current. Left and right are the same. */
  public double getSimTopCurrent() {
    return simLauncherTopCurrent;
  }

  /** Return the bottom simulated current. Left and right are the same. */
  public double getSimBottomCurrent() {
    return simLauncherBottomCurrent;
  }

  @Override
  public void close() {
    // Add closeable objects here
  }
}
