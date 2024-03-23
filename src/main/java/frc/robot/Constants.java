// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotPreferences.PreferenceKeyValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  // Run time options

  // Set to true to log Joystick data. To false otherwise.
  public static final boolean LOG_JOYSTICK_DATA = true;

  // Set to true to send telemetry data to Live Window. To false
  // to disable it.
  public static final boolean LW_TELEMETRY_ENABLE = false;

  // Set a Global Constant to either Show or Hide extended logging data for each of the 5 subsystems
  // Set to true to show extended logging data.
  // Set to false to hide extended logging data.
  public static final boolean SD_SHOW_ARM_EXTENDED_LOGGING_DATA = false;
  public static final boolean SD_SHOW_LAUNCHER_EXTENDED_LOGGING_DATA = false;
  public static final boolean SD_SHOW_CLIMBER_EXTENDED_LOGGING_DATA = false;
  public static final boolean SD_SHOW_INTAKE_EXTENDED_LOGGING_DATA = false;
  public static final boolean SD_SHOW_DRIVE_EXTENDED_LOGGING_DATA = false;

  public static final boolean LOOP_TIMING_LOG = false;

  // Set to true to log each frame of command execution. To false to disable.
  public static final boolean COMMAND_EXECUTE_LOG = false;

  // Camera ID
  public static final int CAMERA_0 = 0;
  public static final int CAMERA_1 = 1;

  // Blinkin
  public static final int BLINKIN_PORT = 1;

  /** Constants used for the Drive subsystem. */
  public static final class DriveConstants {

    private DriveConstants() {
      throw new IllegalStateException("DriveConstants Utility class");
    }

    // Motor Ports
    public static final int FRONT_LEFT_MOTOR_PORT = 2;
    public static final int REAR_LEFT_MOTOR_PORT = 3;
    public static final int FRONT_RIGHT_MOTOR_PORT = 5;
    public static final int REAR_RIGHT_MOTOR_PORT = 4;

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    public static final double GEAR_RATIO = 8.45;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double ENCODER_DISTANCE_METERS_PER_REV =
        // Assumes the encoders are directly mounted on the wheel shafts
        (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO;
    public static final double ENCODER_VELOCITY_CONVERSION =
        (WHEEL_DIAMETER_METERS * Math.PI) / (GEAR_RATIO * 60);

    public static final boolean SQUARE_INPUTS = true;
    public static final boolean ENABLE_BRAKE = false;
    public static final double DEADBAND = 0.05;

    // Constants tunable through preferences
    public static final PreferenceKeyValue DRIVE_TURN_FACTOR =
        new PreferenceKeyValue("DriveTurnFactor", 0.7);
    public static final PreferenceKeyValue DRIVE_SLEW_SPEED =
        new PreferenceKeyValue("DriveSlewSpeed", 2.0);
    public static final PreferenceKeyValue DRIVE_SLEW_TURN =
        new PreferenceKeyValue("DriveSlewTurn", 5.0);
    public static final PreferenceKeyValue DRIVE_NORMAL_SPEED =
        new PreferenceKeyValue("DriveNormalSpeed", 1.0);
    public static final PreferenceKeyValue DRIVE_CRAWL_SPEED =
        new PreferenceKeyValue("DriveCrawlSpeed", 0.3);

    private static final PreferenceKeyValue[] DRIVE_PREFERENCES = {
      DRIVE_TURN_FACTOR, DRIVE_SLEW_SPEED, DRIVE_SLEW_TURN, DRIVE_NORMAL_SPEED, DRIVE_CRAWL_SPEED
    };

    public static PreferenceKeyValue[] getDrivePreferences() {
      return DRIVE_PREFERENCES;
    }
  }

  /** Constants used for the Arm subsystem. */
  public static final class ArmConstants {

    private ArmConstants() {
      throw new IllegalStateException("ArmConstants Utility Class");
    }

    public static final int MOTOR_PORT = 6;
    public static final int BEAM_BREAKER_PORT = 1;

    // Constants tunable through preferences
    public static final PreferenceKeyValue ARM_KP = new PreferenceKeyValue("ArmKP", 6.0);
    public static final PreferenceKeyValue ARM_KS = new PreferenceKeyValue("ArmKS", 0.2);
    public static final PreferenceKeyValue ARM_KG = new PreferenceKeyValue("ArmKG", 0.1);
    public static final PreferenceKeyValue ARM_KV_VOLTS_PER_RAD_PER_SEC =
        new PreferenceKeyValue("ArmKV", 2.0);
    public static final PreferenceKeyValue ARM_MAX_VELOCITY_RAD_PER_SEC =
        new PreferenceKeyValue("ArmVelocityMax", Units.degreesToRadians(120));
    public static final PreferenceKeyValue ARM_MAX_ACCELERATION_RAD_PER_SEC2 =
        new PreferenceKeyValue("ArmAccelerationMax", Units.degreesToRadians(360));

    private static final PreferenceKeyValue[] ARM_PREFERENCES = {
      ARM_KP,
      ARM_KS,
      ARM_KG,
      ARM_KV_VOLTS_PER_RAD_PER_SEC,
      ARM_MAX_VELOCITY_RAD_PER_SEC,
      ARM_MAX_ACCELERATION_RAD_PER_SEC2
    };

    public static PreferenceKeyValue[] getArmPreferences() {
      return ARM_PREFERENCES;
    }

    public static final double GEAR_RATIO = 100;
    public static final double ARM_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI / GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = ARM_RAD_PER_ENCODER_ROTATION / 60;

    // Arm positions.  Horizontal = 0 radians. Assume arm starts at lowest (rest) position
    public static final double ARM_FORWARD_POSITION_RADS = Units.degreesToRadians(-10.0);
    public static final double ARM_BACK_POSITION_RADS = Units.degreesToRadians(170.0);
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-18.0);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(180.0);
    public static final double ARM_OFFSET_RADS = MAX_ANGLE_RADS;
    public static final double POS_INCREMENT = Units.degreesToRadians(2.0); // For small adjustments
    public static final double POSITION_TOLERANCE = Units.degreesToRadians(4.0);
    public static final double VELOCITY_TOLERANCE = Units.degreesToRadians(10.0);
  }

  /** Constants used for the Launcher subsystem. */
  public static final class IntakeConstants {

    private IntakeConstants() {
      throw new IllegalStateException("IntakeConstants Utility Class");
    }

    public static final int INTAKE_MOTOR_PORT = 7;

    // TO DO - update these constants for the real design
    // Constants tunable through preferences
    public static final PreferenceKeyValue INTAKE_KP = new PreferenceKeyValue("IntakeKP", 0.006);
    public static final PreferenceKeyValue INTAKE_KS_VOLTS =
        new PreferenceKeyValue("IntakeKS", 0.0);
    public static final PreferenceKeyValue INTAKE_KV_VOLTS_PER_RPM =
        new PreferenceKeyValue("IntakeKV", 0.025);
    public static final PreferenceKeyValue INTAKE_KA_VOLTS_PER_RPM2 =
        new PreferenceKeyValue("IntakeKA", 0.0);

    public static final PreferenceKeyValue INTAKE_SET_POINT_FORWARD_RPM =
        new PreferenceKeyValue("IntakeForwardRPM", 300.0);
    public static final PreferenceKeyValue INTAKE_SET_POINT_REVERSE_RPM =
        new PreferenceKeyValue("IntakeReverseRPM", -400.0);
    public static final PreferenceKeyValue INTAKE_SPEED_THRESHOLD_RPM =
        new PreferenceKeyValue("IntakeThresholdRPM", 250.0);
    public static final PreferenceKeyValue INTAKE_CURRENT_THRESHOLD_AMPS =
        new PreferenceKeyValue("IntakeThresholdAmps", 8.0);

    private static final PreferenceKeyValue[] INTAKE_PREFERENCES = {
      INTAKE_KP,
      INTAKE_KS_VOLTS,
      INTAKE_KV_VOLTS_PER_RPM,
      INTAKE_KA_VOLTS_PER_RPM2,
      INTAKE_SET_POINT_FORWARD_RPM,
      INTAKE_SET_POINT_REVERSE_RPM,
      INTAKE_SPEED_THRESHOLD_RPM,
      INTAKE_CURRENT_THRESHOLD_AMPS
    };

    public static PreferenceKeyValue[] getIntakePreferences() {
      return INTAKE_PREFERENCES;
    }

    public static final double INTAKE_GEAR_RATIO =
        12.0; // Ratio of motor rotations to output rotations
    public static final double INTAKE_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / INTAKE_GEAR_RATIO;
    public static final double INTAKE_TOLERANCE_RPM = 20;
  }

  /** Constants used for the Launcher subsystem. */
  public static final class LauncherConstants {

    private LauncherConstants() {
      throw new IllegalStateException("LauncherConstants Utility Class");
    }

    public static final int TOP_RIGHT_LAUNCHER_MOTOR_PORT = 12;
    public static final int TOP_LEFT_LAUNCHER_MOTOR_PORT = 13;
    public static final int BOTTOM_RIGHT_LAUNCHER_MOTOR_PORT = 10;
    public static final int BOTTOM_LEFT_LAUNCHER_MOTOR_PORT = 11;

    // TO DO - update these constants for the real design
    // Constants tunable through preferences
    public static final PreferenceKeyValue LAUNCHER_SLEW_VOLTS_PER_SEC =
        new PreferenceKeyValue("LauncherSlew", 24.0);
    public static final PreferenceKeyValue LAUNCHER_KP =
        new PreferenceKeyValue("LauncherKP", 0.001);
    public static final PreferenceKeyValue LAUNCHER_KS_VOLTS =
        new PreferenceKeyValue("LauncherKS", 0.0);
    public static final PreferenceKeyValue LAUNCHER_KV_VOLTS_PER_RPM =
        new PreferenceKeyValue("LauncherKV", 0.002);
    public static final PreferenceKeyValue LAUNCHER_KA_VOLTS_PER_RPM2 =
        new PreferenceKeyValue("LauncherKA", 0.0);

    private static final PreferenceKeyValue[] LAUNCHER_PREFERENCES = {
      LAUNCHER_SLEW_VOLTS_PER_SEC,
      LAUNCHER_KP,
      LAUNCHER_KS_VOLTS,
      LAUNCHER_KV_VOLTS_PER_RPM,
      LAUNCHER_KA_VOLTS_PER_RPM2
    };

    public static PreferenceKeyValue[] getLauncherPreferences() {
      return LAUNCHER_PREFERENCES;
    }

    public static final double LAUNCHER_GEAR_RATIO =
        1.0; // Ratio of motor rotations to output rotations
    public static final double LAUNCHER_ROTATIONS_PER_ENCODER_ROTATION = 1.0 / LAUNCHER_GEAR_RATIO;
    public static final double LAUNCHER_TOLERANCE_RPM = 400;
    public static final double LAUNCHER_TOLERANCE_RPM_AMP = 100;
    public static final double LAUNCHER_TOP_SPEED = 5300;
    public static final double LAUNCHER_BOTTOM_SPEED = 4000;
    public static final double LAUNCHER_TOP_SPEED_AMP = 600;
    public static final double LAUNCHER_BOTTOM_SPEED_AMP = 600;
  }

  /** Constants used for the Climber subsystem. */
  public static final class ClimberConstants {

    private ClimberConstants() {
      throw new IllegalStateException("ClimberConstants Utility Class");
    }

    public static final int LEFT_MOTOR_PORT = 8;
    public static final int RIGHT_MOTOR_PORT = 9;
    public static final int LEFT_RELAY_PORT = 1;
    public static final int RIGHT_RELAY_PORT = 0;

    // TO DO - update these constants for the real design
    // Constants tunable through preferences
    public static final PreferenceKeyValue CLIMBER_KP = new PreferenceKeyValue("ClimberKP", 15.0);
    public static final PreferenceKeyValue CLIMBER_KS = new PreferenceKeyValue("ClimberKS", 0.1);
    public static final PreferenceKeyValue CLIMBER_KG = new PreferenceKeyValue("ClimberKG", 0.55);
    public static final PreferenceKeyValue CLIMBER_KV_VOLTS_PER_METER_PER_SEC =
        new PreferenceKeyValue("ClimberKV", 12.0);
    public static final PreferenceKeyValue CLIMBER_MAX_VELOCITY_METERS_PER_SEC =
        new PreferenceKeyValue("ClimberVelocityMax", 0.2);
    public static final PreferenceKeyValue CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2 =
        new PreferenceKeyValue("ClimberAccelerationMax", 0.5);
    public static final PreferenceKeyValue CLIMBER_FIXED_VOLTS =
        new PreferenceKeyValue("ClimberFixedVolts", 3.0);

    private static final PreferenceKeyValue[] CLIMBER_PREFERENCES = {
      CLIMBER_KP,
      CLIMBER_KS,
      CLIMBER_KG,
      CLIMBER_KV_VOLTS_PER_METER_PER_SEC,
      CLIMBER_MAX_VELOCITY_METERS_PER_SEC,
      CLIMBER_MAX_ACCELERATION_METERS_PER_SEC2,
      CLIMBER_FIXED_VOLTS
    };

    public static PreferenceKeyValue[] getClimberPreferences() {
      return CLIMBER_PREFERENCES;
    }

    public static final double GEAR_RATIO = 16.0; // 16:1 transmission
    // Spool is 1.75 inch diameter, assume additional 0.125 for the cord
    public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(1.875 / 2.0);
    public static final double CLIMBER_METERS_PER_ENCODER_ROTATION =
        2.0 * Math.PI * SPOOL_RADIUS_METERS / GEAR_RATIO;
    public static final double RPM_TO_METERS_PER_SEC = CLIMBER_METERS_PER_ENCODER_ROTATION / 60;
    public static final double CLIMBER_RETRACT_POSITION_METERS = 0.35;
    public static final double CLIMBER_EXTEND_POSITION_METERS = 0.0;
    public static final double CLIMBER_OFFSET_METERS = CLIMBER_RETRACT_POSITION_METERS;

    // Encoder is offset to measure 0 at the top, so minimum pull is 0.
    public static final double CLIMBER_MIN_PULL_METERS = 0.0;
    public static final double CLIMBER_MAX_PULL_METERS = 0.40;

    public static final double POSITION_TOLERANCE_METERS = 0.03;
    public static final double VELOCITY_TOLERANCE_METERS = 0.01;
  }

  /** Constants used for assigning operator input. */
  public static final class OIConstants {

    private OIConstants() {
      throw new IllegalStateException("OIConstants Utility Class");
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
