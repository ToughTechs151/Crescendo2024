// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private CANSparkMaxSim frontLeftSparkSim;
  private CANSparkMaxSim rearLeftSparkSim;
  private CANSparkMaxSim frontRightSparkSim;
  private CANSparkMaxSim rearRightSparkSim;

  private final ADXRS450_GyroSim gyroSim;
  private double lastAngle = 0.0;

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

    // Setup simulation of the CANSparkMax motor controllers and methods to set values
    frontLeftSparkSim = new CANSparkMaxSim(DriveConstants.FRONT_LEFT_MOTOR_PORT);
    rearLeftSparkSim = new CANSparkMaxSim(DriveConstants.REAR_LEFT_MOTOR_PORT);
    frontRightSparkSim = new CANSparkMaxSim(DriveConstants.FRONT_RIGHT_MOTOR_PORT);
    rearRightSparkSim = new CANSparkMaxSim(DriveConstants.REAR_RIGHT_MOTOR_PORT);

    // Set the simulated robot to start at the same position as the real robot.
    drivetrainSimulator.setPose(
        new Pose2d(
            DriveConstants.START_XPOS_METERS,
            DriveConstants.START_YPOS_METERS,
            new Rotation2d(DriveConstants.START_HEADING_RADIANS)));
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void updateSim() {
    // If the drive subsystem odometry has been reset, then reset the simulator to match
    if (driveSubsystem.odometryWasReset()) {
      drivetrainSimulator.setPose(
          new Pose2d(
              DriveConstants.START_XPOS_METERS,
              DriveConstants.START_YPOS_METERS,
              new Rotation2d(DriveConstants.START_HEADING_RADIANS)));

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

    // Set our simulated encoder's position and rate
    double leftSimPosition = drivetrainSimulator.getLeftPositionMeters();
    double rightSimPosition = drivetrainSimulator.getRightPositionMeters();

    frontLeftSparkSim.setPosition(leftSimPosition);
    rearLeftSparkSim.setPosition(leftSimPosition);
    frontRightSparkSim.setPosition(rightSimPosition);
    rearRightSparkSim.setPosition(rightSimPosition);

    double encoderLeftSimRate = drivetrainSimulator.getLeftVelocityMetersPerSecond();
    double encoderRightSimRate = drivetrainSimulator.getRightVelocityMetersPerSecond();

    frontLeftSparkSim.setVelocity(encoderLeftSimRate);
    rearLeftSparkSim.setVelocity(encoderLeftSimRate);
    frontRightSparkSim.setVelocity(encoderRightSimRate);
    rearRightSparkSim.setVelocity(encoderRightSimRate);

    // Set our simulated motor current based on the simulated drivetrain
    double leftSimCurrent = Math.abs(drivetrainSimulator.getLeftCurrentDrawAmps());
    double rightSimCurrent = Math.abs(drivetrainSimulator.getRightCurrentDrawAmps());

    // Current in simulation is total per side so set individual motor current based on number of
    // motors per side.
    frontLeftSparkSim.setCurrent(leftSimCurrent / DriveSimConstants.NUM_MOTORS);
    rearLeftSparkSim.setCurrent(leftSimCurrent / DriveSimConstants.NUM_MOTORS);
    frontRightSparkSim.setCurrent(rightSimCurrent / DriveSimConstants.NUM_MOTORS);
    rearRightSparkSim.setCurrent(rightSimCurrent / DriveSimConstants.NUM_MOTORS);

    // Set gyro angle and rate based on change in angle since last iteration
    double newAngle = -drivetrainSimulator.getHeading().getDegrees();
    gyroSim.setAngle(newAngle);
    gyroSim.setRate(((newAngle - lastAngle) / 0.02));
    lastAngle = newAngle;
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
