package frc.sim;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

/** Constants utility class for the arm simulation. */
public final class Constants {

  private Constants() {
    throw new IllegalStateException("Utility class");
  }

  /** Arm simulation constants. */
  public static final class ArmSim {
    private ArmSim() {
      throw new IllegalStateException("ArmSim Utility Class");
    }

    public static final double ARM_REDUCTION = 1 / ArmConstants.GEAR_RATIO;
    public static final double ARM_MASS_KG = 8.0;
    public static final double ARM_LENGTH_INCHES = 30;
    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(ARM_LENGTH_INCHES);
    public static final double START_ANGLE_RADS = ArmConstants.MIN_ANGLE_RADS;
    public static final int ENCODER_PRR =
        4096; // Only used to simulate noise in position measurement
    public static final double ENCODER_DISTANCE_PER_PULSE =
        2.0 * Math.PI / ENCODER_PRR * ArmConstants.GEAR_RATIO;
  }

  /** Drivetrain simulation constants. */
  public static final class DriveSimConstants {
    private DriveSimConstants() {
      throw new IllegalStateException("ArmSim Utility Class");
    }

    public static final int NUM_MOTORS = 2;
    public static final double KV_LINEAR = 2.0;
    public static final double KA_LINEAR = 0.2;
    public static final double KV_ANGULAR = 3.0;
    public static final double KA_ANGULAR = 0.3;
    // Gain to apply to voltage command to get realistic current (0-1)
    public static final double VOLT_SCALE_FACTOR = 0.7;
  }

  /** Elevator simulation constants. */
  public static final class ElevatorSimConstants {
    private ElevatorSimConstants() {
      throw new IllegalStateException("ElevatorSimConstants Utility Class");
    }

    public static final double ELEVATOR_REDUCTION = 1 / ElevatorConstants.GEAR_RATIO;
    public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);
    public static final double CARRIAGE_MASS = 4.0; // kg
  }

  public static final double POUND_IN2_TO_KG_METERS2 =
      Units.lbsToKilograms(1) * Math.pow(Units.inchesToMeters(1), 2);

  /** Launcher simulation constants. */
  public static final class IntakeSimConstants {
    private IntakeSimConstants() {
      throw new IllegalStateException("IntakeLauncherSimConstants Utility Class");
    }

    public static final double INTAKE_MOI_IN_LBS2 = 1.0;
    public static final double INTAKE_MOI_KG_METERS2 = INTAKE_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
  }
  
  /** Launcher simulation constants. */
  public static final class LauncherSimConstants {
    private LauncherSimConstants() {
      throw new IllegalStateException("IntakeLauncherSimConstants Utility Class");
    }

    public static final double LAUNCHER_MOI_IN_LBS2 = 4.5;
    public static final double LAUNCHER_MOI_KG_METERS2 =
        LAUNCHER_MOI_IN_LBS2 * POUND_IN2_TO_KG_METERS2;
  }
}
