// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotPreferences;

/** Drive subsystem using differential drive. */
public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft =
      new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax rearLeft =
      new CANSparkMax(DriveConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax frontRight =
      new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax rearRight =
      new CANSparkMax(DriveConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  // The front-left-side drive encoder
  private final RelativeEncoder frontLeftEncoder = this.frontLeft.getEncoder();

  // The rear-left-side drive encoder
  private final RelativeEncoder rearLeftEncoder = this.rearLeft.getEncoder();

  // The front-right--side drive encoder
  private final RelativeEncoder frontRightEncoder = this.frontRight.getEncoder();

  // The rear-right-side drive encoder
  private final RelativeEncoder rearRightEncoder = this.rearRight.getEncoder();

  // The gyro sensor
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          this.gyro.getRotation2d(),
          frontLeftEncoder.getPosition(),
          frontRightEncoder.getPosition());

  // Flag to let simulation know when odometry was reset
  boolean odometryReset = false;

  private double normalSpeedMax = 1.0;
  private double crawlSpeedMax = 0.5;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    RobotPreferences.initPreferencesArray(DriveConstants.getDrivePreferences());

    this.frontLeft.restoreFactoryDefaults();
    this.frontRight.restoreFactoryDefaults();
    this.rearLeft.restoreFactoryDefaults();
    this.rearRight.restoreFactoryDefaults();

    // Set the default brake mode to coast and disable the built in deadband since we will apply our
    // own. Set the default drive speed to normal.
    setBrakeMode(false);
    drive.setDeadband(0.0);
    setNormalSpeed();

    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);

    // Sets the distance per pulse for the encoders
    this.frontLeftEncoder.setPositionConversionFactor(
        DriveConstants.ENCODER_DISTANCE_METERS_PER_REV);
    this.rearLeftEncoder.setPositionConversionFactor(
        DriveConstants.ENCODER_DISTANCE_METERS_PER_REV);
    this.frontRightEncoder.setPositionConversionFactor(
        DriveConstants.ENCODER_DISTANCE_METERS_PER_REV);
    this.rearRightEncoder.setPositionConversionFactor(
        DriveConstants.ENCODER_DISTANCE_METERS_PER_REV);
    this.frontLeftEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOCITY_CONVERSION);
    this.rearLeftEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOCITY_CONVERSION);
    this.frontRightEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOCITY_CONVERSION);
    this.rearRightEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_VELOCITY_CONVERSION);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    frontRight.setInverted(true);

    // Set starting pose (position and heading)
    resetOdometry(
        new Pose2d(
            DriveConstants.START_XPOS_METERS,
            DriveConstants.START_YPOS_METERS,
            new Rotation2d(DriveConstants.START_HEADING_RADIANS)));

    SmartDashboard.putData(this.drive);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.odometry.update(
        this.gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());

    SmartDashboard.putNumber("Left pos", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right pos", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("Gyro rate", gyro.getRate());
    // FRONT LEFT
    SmartDashboard.putNumber("FL-Voltage", frontLeft.getBusVoltage());
    SmartDashboard.putNumber("FL-Current", frontLeft.getOutputCurrent());
    SmartDashboard.putNumber("FL-Temp", frontLeft.getMotorTemperature());
    // REAR LEFT
    SmartDashboard.putNumber("RL-Voltage", rearLeft.getBusVoltage());
    SmartDashboard.putNumber("RL-Current", rearLeft.getOutputCurrent());
    SmartDashboard.putNumber("RL-Temp", rearLeft.getMotorTemperature());
    // FRONT RIGHT
    SmartDashboard.putNumber("FR-Voltage", frontRight.getBusVoltage());
    SmartDashboard.putNumber("FR-Current", frontRight.getOutputCurrent());
    SmartDashboard.putNumber("FR-Temp", frontRight.getMotorTemperature());
    // REAR RIGHT
    SmartDashboard.putNumber("RR-Voltage", rearRight.getBusVoltage());
    SmartDashboard.putNumber("RR-Current", rearRight.getOutputCurrent());
    SmartDashboard.putNumber("RR-Temp", rearRight.getMotorTemperature());
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param leftSpeed The left joystick controller speed -1 to 1
   * @param rightSpeed The right joystick controller speed -1 to 1
   * @param squareInputs Enable squaring of the inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
    drive.tankDrive(leftSpeed, rightSpeed, squareInputs);
  }

  /**
   * Arcade drive method for differential drive platform.
   *
   * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
    drive.arcadeDrive(speed, rotation, squareInputs);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  /** Setup the drive command using the tunable settings. */
  public Command getDriveCommand(CommandXboxController driverController) {

    // Read Preferences for the drive speeds
    normalSpeedMax = DriveConstants.DRIVE_NORMAL_SPEED.getValue();
    crawlSpeedMax = DriveConstants.DRIVE_CRAWL_SPEED.getValue();
    setNormalSpeed();

    // Slew rate limiters for joystick inputs (units/sec). For example if the limit=2.0, the input
    // can go from 0 to 1 in 0.5 seconds.
    SlewRateLimiter speedLimiter = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_SPEED.getValue());
    SlewRateLimiter turnLimiter = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_TURN.getValue());

    // A split-stick arcade command, with forward/backward controlled by the left hand, and turn
    // rate controlled by the right. A deadband is applied to both joysticks to avoid creep due to
    // off calibration. Slew rate limits are applied to speed and turn controls. An additional
    // factor is used to desensitize turning.
    return run(() ->
            arcadeDrive(
                -speedLimiter.calculate(
                    MathUtil.applyDeadband(driverController.getLeftY(), DriveConstants.DEADBAND)),
                -DriveConstants.DRIVE_TURN_FACTOR.getValue()
                    * turnLimiter.calculate(
                        MathUtil.applyDeadband(
                            driverController.getRightX(), DriveConstants.DEADBAND)),
                DriveConstants.SQUARE_INPUTS))
        .withName("Arcade");
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public Command driveDistanceCommand(double distanceMeters, double speed, double rot) {
    return
    // Drive forward at specified speed
    run(() -> arcadeDrive(speed, rot, false))
        // End command when we've traveled the specified distance
        .until(() -> getAverageDistanceMeters() >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> drive.stopMotor());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    if (enableBrake) {
      this.frontLeft.setIdleMode(IdleMode.kBrake);
      this.frontRight.setIdleMode(IdleMode.kBrake);
      this.rearLeft.setIdleMode(IdleMode.kBrake);
      this.rearRight.setIdleMode(IdleMode.kBrake);
    } else {
      this.frontLeft.setIdleMode(IdleMode.kCoast);
      this.frontRight.setIdleMode(IdleMode.kCoast);
      this.rearLeft.setIdleMode(IdleMode.kCoast);
      this.rearRight.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Resets the odometry to the specified pose (position and heading).
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    gyro.reset();
    this.odometry.resetPosition(
        this.gyro.getRotation2d(),
        frontLeftEncoder.getPosition(),
        frontRightEncoder.getPosition(),
        pose);

    if (RobotBase.isSimulation()) {
      odometryReset = true;
    }
  }

  /** Returns a Command that resets robot position and heading to the start position. */
  public Command resetOdometryToStart() {
    return runOnce(
            () ->
                resetOdometry(
                    new Pose2d(
                        DriveConstants.START_XPOS_METERS,
                        DriveConstants.START_YPOS_METERS,
                        new Rotation2d(DriveConstants.START_HEADING_RADIANS))))
        .ignoringDisable(true)
        .withName("Reset Start Pose");
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.frontLeftEncoder.setPosition(0);
    this.rearLeftEncoder.setPosition(0);
    this.frontRightEncoder.setPosition(0);
    this.rearRightEncoder.setPosition(0);
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a DifferentialDriveWheelSpeeds object.
   */
  public DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  /** Sets the max output of the drive. Useful for scaling the drive to drive more slowly. */
  public void setNormalSpeed() {
    this.drive.setMaxOutput(normalSpeedMax);
  }

  /** Sets the max output of the drive. Useful for scaling the drive to drive more slowly. */
  public void setCrawlSpeed() {
    this.drive.setMaxOutput(crawlSpeedMax);
  }

  /**
   * Gets the distance the left side wheels have moved since the encoder was last reset.
   *
   * @return the left wheel distance in meters.
   */
  public double getLeftDistanceMeters() {
    return frontLeftEncoder.getPosition();
  }

  /**
   * Gets the distance the right side wheels have moved since the encoder was last reset.
   *
   * @return the right wheel distance in meters.
   */
  public double getRightDistanceMeters() {
    return frontRightEncoder.getPosition();
  }

  /**
   * Gets the average distance the wheels have moved since the encoder was last reset.
   *
   * @return the average wheel distance in meters.
   */
  public double getAverageDistanceMeters() {
    return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -this.gyro.getRate();
  }

  /* The following fields and methods are used during simulation mode.
   *  Get subsystem outputs to the real hardware to drive the simulation
   */

  /**
   * Get a reference to the gyro for simulation.
   *
   * @return Reference to the gyro device
   */
  public ADXRS450_Gyro getGyro() {
    return gyro;
  }

  /**
   * Disable the drive by setting motor output to zero. Any PID controllers should also be disabled
   * here. NOTE: In this state the drive will roll to a stop if using coast mode. Using EMF braking
   * mode will cause drive to stop quickly.
   */
  public void disable() {
    frontLeft.setVoltage(0);
    frontRight.setVoltage(0);
  }

  // The following methods are used for the simulation to get drive state

  /**
   * Get the voltage command to the left motor.
   *
   * @return command to the left motor controller group in volts
   */
  public double getLeftMotorVolts() {
    return frontLeft.get();
  }

  /**
   * Get the voltage command to the right motor.
   *
   * @return command to the right motor controller group in volts
   */
  public double getRightMotorVolts() {
    return frontRight.get();
  }

  /**
   * Get the state of odometry reset. This is used by simulation to determine when the drive model
   * also needs to be reset to match.
   *
   * @return a flag indicating odometry was reset
   */
  public boolean odometryWasReset() {
    return odometryReset;
  }

  /** Clear the state of odometry reset. */
  public void clearOdometryReset() {
    odometryReset = false;
  }
}
