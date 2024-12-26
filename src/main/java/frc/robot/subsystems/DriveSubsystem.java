// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotPreferences;
import frc.robot.StartPose;
import frc.robot.StartPose.NamedPose;
import java.util.Map;

/** Drive subsystem using differential drive. */
public class DriveSubsystem extends SubsystemBase {
  private final SparkMax frontLeft =
      new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax rearLeft =
      new SparkMax(DriveConstants.REAR_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax frontRight =
      new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private final SparkMax rearRight =
      new SparkMax(DriveConstants.REAR_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  // The front-left-side drive encoder
  private final RelativeEncoder frontLeftEncoder = this.frontLeft.getEncoder();

  // The rear-left-side drive encoder
  private final RelativeEncoder rearLeftEncoder = this.rearLeft.getEncoder();

  // The front-right--side drive encoder
  private final RelativeEncoder frontRightEncoder = this.frontRight.getEncoder();

  // The rear-right-side drive encoder
  private final RelativeEncoder rearRightEncoder = this.rearRight.getEncoder();

  private final SparkMaxConfig globalConfig = new SparkMaxConfig();
  private final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

  // The gyro sensor
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(
          this.gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

  // Flag to let simulation know when odometry was reset
  boolean odometryReset = false;

  private double normalSpeedMax = 1.0;
  private double crawlSpeedMax = 0.5;

  private final SendableChooser<Integer> startPoseChooser = new SendableChooser<>();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    RobotPreferences.initPreferencesArray(DriveConstants.getDrivePreferences());

    // Common motor / encoder settings
    globalConfig.smartCurrentLimit(DriveConstants.CURRENT_LIMIT).idleMode(IdleMode.kCoast);

    globalConfig.encoder.velocityConversionFactor(DriveConstants.RPM_TO_METERS_PER_SEC);

    // .encoder.positionConversionFactor(DriveConstants.METERS_PER_ENCODER_REV);
    // Not working in 2025 Beta 3 for simulation

    // Unique settings per position. Set followers and invert right side so that positive voltages
    // result in both sides moving forward.
    leftFollowerConfig.apply(globalConfig).follow(frontLeft);

    rightLeaderConfig.apply(globalConfig).inverted(true);

    rightFollowerConfig.apply(globalConfig).follow(frontRight);

    // Set the configurations for the motor controllers
    frontLeft.configure(
        globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rearLeft.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    frontRight.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rearRight.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    frontLeft.clearFaults();
    rearLeft.clearFaults();
    frontRight.clearFaults();
    rearRight.clearFaults();

    frontLeftEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);

    // Disable the built in deadband since we will apply our own.
    // Set the default drive speed to normal.
    drive.setDeadband(0.0);
    setNormalSpeed();

    // Set starting pose (position and heading)
    setupStartPoseChooser();
    resetOdometry();

    SmartDashboard.putData(this.drive);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.odometry.update(
        this.gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

    SmartDashboard.putNumber("Drive FL-Position", getLeftDistanceMeters());
    SmartDashboard.putNumber("Drive FR-Position", getRightDistanceMeters());
    SmartDashboard.putNumber("Drive FL-Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive FR-Velocity", frontRightEncoder.getVelocity());

    SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("Gyro rate", gyro.getRate());
    // FRONT LEFT
    SmartDashboard.putNumber(
        "Drive FL-Voltage", frontLeft.getAppliedOutput() * frontLeft.getBusVoltage());
    SmartDashboard.putNumber("Drive FL-Current", frontLeft.getOutputCurrent());
    SmartDashboard.putNumber("Drive FL-Temp", frontLeft.getMotorTemperature());
    // REAR LEFT
    SmartDashboard.putNumber(
        "Drive RL-Voltage", rearLeft.getAppliedOutput() * rearLeft.getBusVoltage());
    SmartDashboard.putNumber("Drive RL-Current", rearLeft.getOutputCurrent());
    SmartDashboard.putNumber("Drive RL-Temp", rearLeft.getMotorTemperature());
    // FRONT RIGHT
    SmartDashboard.putNumber(
        "Drive FR-Voltage", frontRight.getAppliedOutput() * frontRight.getBusVoltage());
    SmartDashboard.putNumber("Drive FR-Current", frontRight.getOutputCurrent());
    SmartDashboard.putNumber("Drive FR-Temp", frontRight.getMotorTemperature());
    // REAR RIGHT
    SmartDashboard.putNumber(
        "Drive RR-Voltage", rearRight.getAppliedOutput() * rearRight.getBusVoltage());
    SmartDashboard.putNumber("Drive RR-Current", rearRight.getOutputCurrent());
    SmartDashboard.putNumber("Drive RR-Temp", rearRight.getMotorTemperature());
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
    // disable driving until follow is fixed
    if (RobotBase.isSimulation()) {
      drive.arcadeDrive(speed, rotation, squareInputs);
    }
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
   * Returns a command that drives the robot forward to a specified position at a specified speed.
   *
   * @param stopPositionMeters The position in meters at which to stop
   * @param speed The fraction of max speed at which to drive
   */
  public Command driveForwardCommand(double stopPositionMeters, double speed, double rot) {
    return run(() -> arcadeDrive(speed, rot, false))
        .until(() -> getAverageDistanceMeters() >= stopPositionMeters)
        .finallyDo(interrupted -> drive.stopMotor());
  }

  /**
   * Returns a command that drives the robot in reverse to a specified position at a specified
   * speed.
   *
   * @param stopPositionMeters The position in meters at which to stop
   * @param speed The fraction of max speed at which to drive
   */
  public Command driveReverseCommand(double stopPositionMeters, double speed, double rot) {
    return run(() -> arcadeDrive(-speed, rot, false))
        .until(() -> getAverageDistanceMeters() < stopPositionMeters)
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
    return new DifferentialDriveWheelSpeeds(getLeftDistanceMeters(), getRightDistanceMeters());
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    if (enableBrake) {
      DataLogManager.log("Drive is currently set to brake mode");
      brakeConfig.idleMode(IdleMode.kBrake);

    } else {
      DataLogManager.log("Drive is currently set to coast mode");
      brakeConfig.idleMode(IdleMode.kCoast);
    }
    frontLeft.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    frontRight.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rearLeft.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rearLeft.configure(
        brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Setup the options for the starting position chooser. */
  private void setupStartPoseChooser() {

    // Add the list of start poses to the chooser
    NamedPose[] poseArray = StartPose.get();

    startPoseChooser.setDefaultOption(poseArray[0].name(), 0);
    for (int index = 1; index < poseArray.length; index++) {
      startPoseChooser.addOption(poseArray[index].name(), index);
    }

    // Put the chooser on the Shuffleboard Driver tab
    ShuffleboardLayout startPoseChooserLayout =
        Shuffleboard.getTab("Driver")
            .getLayout("Start Pose", BuiltInLayouts.kList)
            .withSize(3, 1)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "HIDDEN"));
    startPoseChooserLayout.add(startPoseChooser);
  }

  /** Resets the odometry to the specified pose (position and heading). */
  public void resetOdometry() {

    resetEncoders();
    gyro.reset();

    this.odometry.resetPosition(
        this.gyro.getRotation2d(),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        getStartPose());

    if (RobotBase.isSimulation()) {
      odometryReset = true;
    }
  }

  /** Get the selected starting pose from the chooser. */
  public Pose2d getStartPose() {

    NamedPose pose = StartPose.get()[startPoseChooser.getSelected()];

    return new Pose2d(pose.x(), pose.y(), new Rotation2d(Units.degreesToRadians(pose.heading())));
  }

  /** Returns a Command that resets robot position and heading to the start position. */
  public Command resetOdometryToStart() {
    return runOnce(this::resetOdometry).ignoringDisable(true).withName("Reset Start Pose");
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
    return frontLeftEncoder.getPosition() * DriveConstants.METERS_PER_ENCODER_REV;
    // scale factor workaround for 2025 Beta 3
  }

  /**
   * Gets the distance the right side wheels have moved since the encoder was last reset.
   *
   * @return the right wheel distance in meters.
   */
  public double getRightDistanceMeters() {
    return frontRightEncoder.getPosition() * DriveConstants.METERS_PER_ENCODER_REV;
    // scale factor workaround for 2025 Beta 3
  }

  /**
   * Gets the average distance the wheels have moved since the encoder was last reset.
   *
   * @return the average wheel distance in meters.
   */
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
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

  /** Returns the front left motor for simulation. */
  public SparkMax getFrontLeftMotor() {
    return frontLeft;
  }

  /** Returns the rear left motor for simulation. */
  public SparkMax getRearLeftMotor() {
    return rearLeft;
  }

  /** Returns the front right motor for simulation. */
  public SparkMax getFrontRightMotor() {
    return frontRight;
  }

  /** Returns the rear right motor for simulation. */
  public SparkMax getRearRightMotor() {
    return rearRight;
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
