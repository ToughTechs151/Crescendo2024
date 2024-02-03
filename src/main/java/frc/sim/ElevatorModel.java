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
import frc.sim.Constants.ElevatorSimConstants;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ElevatorModel implements AutoCloseable {

  private final ClimberSubsystem elevatorSubsystem;
  private double simCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor elevatorGearbox = DCMotor.getMiniCIM(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This elevator sim represents an elevator that can travel up and down when driven by the motor
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorGearbox,
          ElevatorSimConstants.ELEVATOR_REDUCTION,
          ElevatorSimConstants.CARRIAGE_MASS,
          ElevatorSimConstants.ELEVATOR_DRUM_RADIUS,
          ClimberConstants.CLIMBER_MIN_HEIGHT_METERS,
          ClimberConstants.CLIMBER_MAX_HEIGHT_METERS,
          true,
          0,
          VecBuilder.fill(0.002));

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 1.0, 0.5);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  /** Create a new ElevatorModel. */
  public ElevatorModel(ClimberSubsystem elevatorSubsystemToSimulate) {

    elevatorSubsystem = elevatorSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(ClimberConstants.MOTOR_PORT);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(elevatorSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.setPosition(elevatorSim.getPositionMeters());
    sparkSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());
    simCurrent = elevatorSim.getCurrentDrawAmps();
    sparkSim.setCurrent(simCurrent);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with position
    elevatorMech2d.setLength(elevatorSim.getPositionMeters());
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simCurrent;
  }

  @Override
  public void close() {
    mech2d.close();
    elevatorMech2d.close();
  }
}
