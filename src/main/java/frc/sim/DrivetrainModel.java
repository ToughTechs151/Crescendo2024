// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.sim.Constants.DriveSimConstants;

/** Model of a differential drivetrain. */
public class DrivetrainModel {

  private final DriveSubsystem driveSubsystem;

  private SparkMaxSim frontLeftSparkSim;
  private SparkMaxSim rearLeftSparkSim;
  private SparkMaxSim frontRightSparkSim;
  private SparkMaxSim rearRightSparkSim;

  private final ADXRS450_GyroSim gyroSim;
  private double lastAngle = 0.0;
  private double startAngle = 0.0;

  // Range of pose positions within the field boundary (meters)
  private static double fieldMinX = 0.5;
  private static double fieldMaxX = 16.3;
  private static double fieldMinY = 0.6;
  private static double fieldMaxY = 7.8;

  private final LinearSystem<N2, N2, N2> drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(
          DriveSimConstants.KV_LINEAR,
          DriveSimConstants.KA_LINEAR,
          DriveSimConstants.KV_ANGULAR,
          DriveSimConstants.KA_ANGULAR);

  private final LimitedDifferentialDriveSim drivetrainSimulator =
      new LimitedDifferentialDriveSim(
          drivetrainSystem,
          DCMotor.getNEO(DriveSimConstants.NUM_MOTORS),
          8,
          DriveConstants.TRACK_WIDTH_METERS,
          DriveConstants.WHEEL_DIAMETER_METERS / 2.0, // Wheel Radius
          null,
          fieldMinX,
          fieldMaxX,
          fieldMinY,
          fieldMaxY);

  /** Subsystem constructor. */
  public DrivetrainModel(DriveSubsystem driveSubsystemToSimulate) {
    driveSubsystem = driveSubsystemToSimulate;
    gyroSim = new ADXRS450_GyroSim(driveSubsystem.getGyro());

    simulationInit();
  }

  /** Initialize the drivetrain simulation. */
  public void simulationInit() {

    // Setup simulation of the SparkMax motor controllers and methods to set values
    DCMotor motorParameters = DCMotor.getNEO(1);
    frontLeftSparkSim = new SparkMaxSim(driveSubsystem.getFrontLeftMotor(), motorParameters);
    rearLeftSparkSim = new SparkMaxSim(driveSubsystem.getRearLeftMotor(), motorParameters);
    frontRightSparkSim = new SparkMaxSim(driveSubsystem.getFrontRightMotor(), motorParameters);
    rearRightSparkSim = new SparkMaxSim(driveSubsystem.getRearRightMotor(), motorParameters);

    // Set the simulated robot to start at the same position as the real robot.
    setStartPose();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void updateSim() {
    // If the drive subsystem odometry has been reset, then reset the simulator to match
    if (driveSubsystem.odometryWasReset()) {
      setStartPose();
      driveSubsystem.clearOdometryReset();

    } else {
      // To update our simulation, we set motor voltage inputs, update the
      // simulation, and write the simulated positions and velocities to our
      // simulated encoder and gyro. We negate the right side so that positive
      // voltages make the right side move forward.
      drivetrainSimulator.setInputs(
          driveSubsystem.getLeftMotorVolts()
              * RobotController.getInputVoltage()
              * DriveSimConstants.VOLT_SCALE_FACTOR,
          driveSubsystem.getRightMotorVolts()
              * RobotController.getInputVoltage()
              * DriveSimConstants.VOLT_SCALE_FACTOR);

      drivetrainSimulator.update(0.02);
    }

    // Finally, we run the spark simulation
    frontLeftSparkSim.iterate(drivetrainSimulator.getLeftVelocityMetersPerSecond(), 12.0, 0.02);
    rearLeftSparkSim.iterate(drivetrainSimulator.getLeftVelocityMetersPerSecond(), 12.0, 0.02);
    frontRightSparkSim.iterate(drivetrainSimulator.getRightVelocityMetersPerSecond(), 12.0, 0.02);
    rearRightSparkSim.iterate(drivetrainSimulator.getRightVelocityMetersPerSecond(), 12.0, 0.02);

    // SmartDashboard.putNumber("Dr Sim L Pos", drivetrainSimulator.getLeftPositionMeters());
    // SmartDashboard.putNumber("Dr Sim R Pos", drivetrainSimulator.getRightPositionMeters());
    // SmartDashboard.putNumber("Dr Sim L Vel",
    // drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // SmartDashboard.putNumber("Dr Sim R Vel",
    // drivetrainSimulator.getRightVelocityMetersPerSecond());

    // Set gyro angle with offset from the angle at last reset. Set the rate based on change in
    // angle since last iteration.
    double newAngle = -(drivetrainSimulator.getHeading().getDegrees() - startAngle);
    gyroSim.setAngle(newAngle);
    gyroSim.setRate(((newAngle - lastAngle) / 0.02));
    lastAngle = newAngle;
  }

  private void setStartPose() {
    Pose2d newPose = driveSubsystem.getStartPose();
    drivetrainSimulator.setPose(newPose);

    // The gyro is reset to zero, so save the offset from the starting heading
    startAngle = newPose.getRotation().getDegrees();
  }

  /** Return the left side total simulated current. */
  public double getLeftSimCurrent() {
    return drivetrainSimulator.getLeftCurrentDrawAmps();
  }

  /** Return the right side total simulated current. */
  public double getRightSimCurrent() {
    return drivetrainSimulator.getRightCurrentDrawAmps();
  }
}
