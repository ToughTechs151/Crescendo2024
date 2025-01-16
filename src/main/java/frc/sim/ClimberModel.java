// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
  private double simCurrentLeft = 0.0;
  private double simCurrentRight = 0.0;
  private SparkMaxSim sparkLeftSim;
  private SparkMaxSim sparkRightSim;

  // The arm gearbox represents a gearbox containing one motor.
  private final DCMotor climberGearbox = DCMotor.getNEO(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This climber sim models an elevator that can travel up and down when driven by the motor.
  // The sim is for an elevator being pulled up, but we are using it to represent pulling
  // the robot up. "Height" actually means how far the robot is pulled up.
  private final ElevatorSim climberLeftSim =
      new ElevatorSim(
          climberGearbox,
          ClimberConstants.GEAR_RATIO,
          ClimberSimConstants.CARRIAGE_MASS,
          ClimberConstants.SPOOL_RADIUS_METERS,
          ClimberConstants.CLIMBER_MIN_PULL_METERS,
          ClimberConstants.CLIMBER_MAX_PULL_METERS,
          true,
          ClimberConstants.CLIMBER_RETRACT_POSITION_METERS,
          0.0005,
          0.0);

  private final ElevatorSim climberRightSim =
      new ElevatorSim(
          climberGearbox,
          ClimberConstants.GEAR_RATIO,
          ClimberSimConstants.CARRIAGE_MASS,
          ClimberConstants.SPOOL_RADIUS_METERS,
          ClimberConstants.CLIMBER_MIN_PULL_METERS,
          ClimberConstants.CLIMBER_MAX_PULL_METERS,
          true,
          ClimberConstants.CLIMBER_RETRACT_POSITION_METERS,
          0.0005,
          0.0);

  // Create Mechanism2d visualizations of the climber mechanisms
  private final Mechanism2d mech2dLeft = new Mechanism2d(1, 1.2);
  private final Mechanism2d mech2dRight = new Mechanism2d(1, 1.2);
  private final MechanismRoot2d mech2dRootLeft = mech2dLeft.getRoot("Climber Root", 0.5, 0.2);
  private final MechanismRoot2d mech2dRootRight = mech2dRight.getRoot("Climber Root", 0.5, 0.2);
  private final MechanismLigament2d climberMech2dLeft =
      mech2dRootLeft.append(
          new MechanismLigament2d(
              "Climber Left",
              ClimberConstants.CLIMBER_MAX_PULL_METERS - climberLeftSim.getPositionMeters(),
              90));
  private final MechanismLigament2d climberMech2dRight =
      mech2dRootRight.append(
          new MechanismLigament2d(
              "Climber Right",
              ClimberConstants.CLIMBER_MAX_PULL_METERS - climberRightSim.getPositionMeters(),
              90));

  /** Create a new Climber Model. */
  public ClimberModel(ClimberSubsystem climberSubsystemToSimulate) {

    climberSubsystem = climberSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    // To view the Climber visualization, select:
    //    Network Tables -> SmartDashboard -> Climber Left Sim
    //    Network Tables -> SmartDashboard -> Climber Right Sim
    SmartDashboard.putData("Climber Left Sim", mech2dLeft);
    SmartDashboard.putData("Climber Right Sim", mech2dRight);
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup simulations of the CANSparkMax and methods to set values
    sparkLeftSim = new SparkMaxSim(climberSubsystem.getLeftMotor(), climberGearbox);
    sparkRightSim = new SparkMaxSim(climberSubsystem.getRightMotor(), climberGearbox);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our climbers are doing
    // First, we set our "inputs" (voltages)
    climberLeftSim.setInput(climberSubsystem.getLeftVoltageCommand());
    climberRightSim.setInput(climberSubsystem.getRightVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    climberLeftSim.update(0.020);
    climberRightSim.update(0.020);

    // Finally, we  run the spark simulations, set our simulated encoder's readings and save the
    // current so it can be retrieved later.
    sparkLeftSim.iterate(climberLeftSim.getVelocityMetersPerSecond(), 12.0, 0.02);
    sparkRightSim.iterate(climberRightSim.getVelocityMetersPerSecond(), 12.0, 0.02);

    simCurrentLeft = Math.abs(climberLeftSim.getCurrentDrawAmps());
    simCurrentRight = Math.abs(climberRightSim.getCurrentDrawAmps());

    // Update climber visualizations with position
    climberMech2dLeft.setLength(
        ClimberConstants.CLIMBER_MAX_PULL_METERS - climberLeftSim.getPositionMeters());
    climberMech2dRight.setLength(
        ClimberConstants.CLIMBER_MAX_PULL_METERS - climberRightSim.getPositionMeters());
  }

  /** Return the left side simulated current. */
  public double getSimCurrentLeft() {
    return simCurrentLeft;
  }

  /** Return the right side simulated current. */
  public double getSimCurrentRight() {
    return simCurrentRight;
  }

  @Override
  public void close() {
    mech2dLeft.close();
    mech2dRight.close();
    climberMech2dLeft.close();
    climberMech2dRight.close();
  }
}
