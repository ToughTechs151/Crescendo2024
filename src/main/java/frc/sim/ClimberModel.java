// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.sim.Constants.ClimberSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ClimberModel implements AutoCloseable {

  private final ClimberSubsystem climberSubsystem;
  private double simCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor climberGearbox = DCMotor.getMiniCIM(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This climber sim models an elevator that can travel up and down when driven by the motor
  private final ElevatorSim climberSim =
      new ElevatorSim(
          climberGearbox,
          ClimberSimConstants.CLIMBER_REDUCTION,
          ClimberSimConstants.CARRIAGE_MASS,
          ClimberSimConstants.CLIMBER_DRUM_RADIUS,
          ClimberConstants.CLIMBER_MIN_HEIGHT_METERS,
          ClimberConstants.CLIMBER_MAX_HEIGHT_METERS,
          true,
          0,
          VecBuilder.fill(0.002));

  // Create a Mechanism2d visualization of the climber
  private final Mechanism2d mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Climber Root", 1.0, 0.5);
  private final MechanismLigament2d climberMech2d =
      mech2dRoot.append(new MechanismLigament2d("Climber", climberSim.getPositionMeters(), 90));

  /** Create a new Climber Model. */
  public ClimberModel(ClimberSubsystem climberSubsystemToSimulate) {

    climberSubsystem = climberSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Climber visualization, select Network Tables -> SmartDashboard -> Climber Sim
    SmartDashboard.putData("Climber Sim", mech2d);
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(ClimberConstants.LEFT_MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    climberSim.setInput(climberSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    climberSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.setPosition(climberSim.getPositionMeters());
    sparkSim.setVelocity(climberSim.getVelocityMetersPerSecond());
    simCurrent = climberSim.getCurrentDrawAmps();
    sparkSim.setCurrent(simCurrent);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    // Update climber visualization with position
    climberMech2d.setLength(climberSim.getPositionMeters());
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simCurrent;
  }

  @Override
  public void close() {
    mech2d.close();
    climberMech2d.close();
  }
}
