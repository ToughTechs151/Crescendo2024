package frc.robot;

// Forked from FRC Team 2832 "The Livonia Warriors"

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import java.util.Map;

/** The DataLogging class contains all the logic for using telemetry. */
public class DataLogging {

  private DoubleLogEntry loopTime;
  private double startTime;
  private ShuffleboardTab sbCommandsTab;
  private ShuffleboardLayout pdpWidget;
  private boolean everBrownout = false;
  private boolean prevDsConnectState;
  private ShuffleboardTab sbDriverTab;
  private Field2d sbField;
  private DriveSubsystem drive;
  private ArmSubsystem arm;
  private BlinkinSubsystem blinkin;

  private DataLogging() {
    // Starts recording to data log
    DataLogManager.start();
    final DataLog log = DataLogManager.getLog();

    // Record the starting values of preferences
    DataLogManager.log("Starting Preference Values:");
    RobotPreferences.logPreferences();

    // Record both DS control and joystick data. To
    DriverStation.startDataLog(DataLogManager.getLog(), Constants.LOG_JOYSTICK_DATA);

    if (Constants.LW_TELEMETRY_ENABLE) {
      // In 2022 code this is the default, in 2023 the default changes
      // and they add the enable call.
      // LiveWindow.enableAllTelemetry()
    } else {
      LiveWindow.disableAllTelemetry();
    }

    ShuffleboardTab sbRobotTab = Shuffleboard.getTab("Robot");
    pdpWidget = sbRobotTab.getLayout("PDP", BuiltInLayouts.kGrid).withSize(3, 4).withPosition(3, 0);
    ShuffleboardLayout rcWidget =
        sbRobotTab.getLayout("RobotController", BuiltInLayouts.kGrid).withSize(3, 3);

    sbCommandsTab = Shuffleboard.getTab("Commands");

    /* sbRobotTab */
    rcWidget
        .addNumber("Batt Volt", RobotController::getBatteryVoltage)
        .withWidget(BuiltInWidgets.kVoltageView)
        .withProperties(Map.of("min", 0, "max", 13));
    rcWidget
        .addBoolean("Brown Out", RobotController::isBrownedOut)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Green"));
    rcWidget
        .addBoolean("Ever Browned Out", this::getEverBrownOut)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Green"));

    prevDsConnectState = DriverStation.isDSAttached();

    /* Drivers tab */
    sbDriverTab = Shuffleboard.getTab("Driver");

    DataLogManager.log(String.format("Brownout Voltage: %f", RobotController.getBrownoutVoltage()));

    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish

    StringLogEntry commandLog = new StringLogEntry(log, "/command/event");
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> commandLog.append("Command initialized:" + command.getName()));

    if (Constants.COMMAND_EXECUTE_LOG) {
      CommandScheduler.getInstance()
          .onCommandExecute(command -> commandLog.append("Command execute:" + command.getName()));
    }

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> commandLog.append("Command interrupted:" + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> commandLog.append("Command finished:" + command.getName()));
    commandLog.append("Opened command log");

    loopTime = new DoubleLogEntry(log, "/robot/LoopTime");
  }

  private static class InstanceHolder {
    private static final DataLogging instance = new DataLogging();
  }

  /**
   * Gets the datalogging Singleton object.
   *
   * @return DataLogging
   */
  public static DataLogging getInstance() {
    return InstanceHolder.instance;
  }

  /**
   * Runs at each loop slice.. This method should be called in the robotPeriodic method in
   * Robot.java. the code must be the last thing in the method.
   *
   * <pre>{@code
   * //must be at end
   * datalog.periodic();
   * }</pre>
   */
  public void periodic() {

    if (RobotController.isBrownedOut()) {
      everBrownout = true;
    }

    boolean newDsConnectState = DriverStation.isDSAttached();
    if (prevDsConnectState != newDsConnectState) {
      Shuffleboard.addEventMarker(
          "Driver Station is %s" + (newDsConnectState ? "Connected" : "Disconnected"),
          EventImportance.kHigh);
      prevDsConnectState = newDsConnectState;
    }

    // Get the pose from the drivetrain subsystem and update the field display
    sbField.setRobotPose(drive.getPose());

    // Set the LEDs to show arm angle
    if (arm.getMeasurement() > Constants.ArmConstants.ARM_BACK_POSITION_RADS) {
      blinkin.setValue(BlinkinSubsystem.RED);
    } else if (arm.getMeasurement() < Constants.ArmConstants.ARM_FORWARD_POSITION_RADS) {
      if (arm.IsNoteInsideIntake()) {
        blinkin.setValue(BlinkinSubsystem.ORANGE);
      } else {
        blinkin.setValue(BlinkinSubsystem.BLUE);
      }
    } else if (arm.getMeasurement() < Constants.ArmConstants.ARM_BACK_POSITION_RADS
        && arm.getMeasurement() > Constants.ArmConstants.ARM_FORWARD_POSITION_RADS) {
      blinkin.setValue(BlinkinSubsystem.ORANGE);
    }

    if (Constants.LOOP_TIMING_LOG) {
      loopTime.append(Timer.getFPGATimestamp() - startTime);
    }
  }

  /**
   * Called from robot.java immediately after the robotContainer is created.
   *
   * @param robotContainer The robotContainer just constructed.
   */
  public void dataLogRobotContainerInit(RobotContainer robotContainer) {

    blinkin = robotContainer.getBlinkin();
    drive = robotContainer.getDriveSubsystem();
    arm = robotContainer.getArmSubsystem();
    ClimberSubsystem climber = robotContainer.getClimberSubsystem();
    IntakeSubsystem intake = robotContainer.getIntakeSubsystem();
    LauncherSubsystem launcher = robotContainer.getLauncherSubsystem();

    // Add widgets to the Commands tab
    sbCommandsTab.add(arm).withSize(3, 1).withPosition(3, 0);
    sbCommandsTab.add(climber).withSize(3, 1).withPosition(3, 1);
    sbCommandsTab.add(drive).withSize(3, 1).withPosition(3, 2);
    sbCommandsTab.add(intake).withSize(3, 1).withPosition(3, 3);
    sbCommandsTab.add(launcher).withSize(3, 1).withPosition(3, 4);

    ShuffleboardLayout resetPreferencesLayout =
        sbCommandsTab
            .getLayout("Reset Preferences", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "HIDDEN"));

    // Add buttons to reset preferences to the default constant values
    resetPreferencesLayout
        .add(
            new InstantCommand(
                    () ->
                        RobotPreferences.resetPreferencesArray(
                            Constants.ArmConstants.getArmPreferences()))
                .ignoringDisable(true)
                .withName("Reset Arm Preferences"))
        .withSize(2, 1);

    resetPreferencesLayout
        .add(
            new InstantCommand(
                    () ->
                        RobotPreferences.resetPreferencesArray(
                            Constants.ClimberConstants.getClimberPreferences()))
                .ignoringDisable(true)
                .withName("Reset Climber Preferences"))
        .withSize(2, 1);

    resetPreferencesLayout
        .add(
            new InstantCommand(
                    () ->
                        RobotPreferences.resetPreferencesArray(
                            Constants.DriveConstants.getDrivePreferences()))
                .ignoringDisable(true)
                .withName("Reset Drive Preferences"))
        .withSize(2, 1);

    resetPreferencesLayout
        .add(
            new InstantCommand(
                    () ->
                        RobotPreferences.resetPreferencesArray(
                            Constants.IntakeConstants.getIntakePreferences()))
                .ignoringDisable(true)
                .withName("Reset Intake Preferences"))
        .withSize(2, 1);

    resetPreferencesLayout
        .add(
            new InstantCommand(
                    () ->
                        RobotPreferences.resetPreferencesArray(
                            Constants.LauncherConstants.getLauncherPreferences()))
                .ignoringDisable(true)
                .withName("Reset Launcher Preferences"))
        .withSize(2, 1);

    resetPreferencesLayout
        .add(
            new InstantCommand(RobotPreferences::resetAllPreferences)
                .ignoringDisable(true)
                .withName("Reset All Preferences"))
        .withSize(2, 1);

    // Add the chooser to select the autonomous mode command
    SendableChooser<String> autoChooser = robotContainer.getAutoChooser();
    ShuffleboardLayout autoChooserLayout =
        sbCommandsTab
            .getLayout("Autonomous Command", BuiltInLayouts.kList)
            .withSize(3, 1)
            .withPosition(6, 0)
            .withProperties(Map.of("Label position", "HIDDEN"));
    autoChooserLayout.add(autoChooser);

    // Add widgets to the Driver tab to display the robot pose and a button to run the Reset
    // Start Pose command.
    sbField = new Field2d();
    sbDriverTab.add("Field", sbField).withSize(8, 4);
    sbDriverTab.add(drive.resetOdometryToStart()).withSize(2, 1);

    // Add hardware sendables here
    PowerDistribution pdp = robotContainer.getPdp();
    pdpWidget.add("PDP", pdp);

    // Log configuration info here
    DataLogManager.log(String.format("PDP Can ID: %d", pdp.getModule()));

    // Add values with supplier functions here.
    pdpWidget
        .addNumber("PDP Temp", pdp::getTemperature)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 15, "max", 50));

    // HW Test tab for buttons to control hardware for testing and maintenance
    ShuffleboardTab sbHardwareTestTab = Shuffleboard.getTab("HW Test");

    sbHardwareTestTab
        .add(
            new InstantCommand(() -> arm.setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Arm Brake Mode"))
        .withSize(2, 1)
        .withPosition(0, 0);

    sbHardwareTestTab
        .add(
            new InstantCommand(() -> arm.setBrakeMode(false))
                .ignoringDisable(true)
                .withName("Arm Coast Mode"))
        .withSize(2, 1)
        .withPosition(0, 1);

    sbHardwareTestTab
        .add(
            new InstantCommand(arm::resetEncoder)
                .ignoringDisable(true)
                .withName("Arm Reset Position"))
        .withSize(2, 1)
        .withPosition(0, 2);

    sbHardwareTestTab
        .add(
            new InstantCommand(() -> climber.setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Climber Brake Mode"))
        .withSize(2, 1)
        .withPosition(3, 0);

    sbHardwareTestTab
        .add(
            new InstantCommand(() -> climber.setBrakeMode(false))
                .ignoringDisable(true)
                .withName("Climber Coast Mode"))
        .withSize(2, 1)
        .withPosition(3, 1);

    sbHardwareTestTab
        .add(
            new InstantCommand(climber::resetEncoders)
                .ignoringDisable(true)
                .withName("Climber Reset Position"))
        .withSize(2, 1)
        .withPosition(3, 2);
  }

  /**
   * Add a button for the command onto the Shuffleboard Commands tab.
   *
   * @param comName The name of the command.
   * @param com The command object.
   */
  public void logCommand(String comName, Sendable com) {
    sbCommandsTab.add(comName, com).withSize(2, 1);
  }

  /**
   * Add a button for the command in the subsystem group. Usually called in the command constructor.
   *
   * <pre>{@code
   * DataLogging.getInstance().logCommand(this.subsystem.getName(),
   * this.getName(), this);
   * }</pre>
   *
   * @param ssName The name of subsystem.
   * @param comName The name of the command.
   * @param com The command object.
   */
  public final void logCommand(String ssName, String comName, Sendable com) {
    sbCommandsTab.getLayout(ssName, BuiltInLayouts.kList).withSize(2, 0).add(comName, com);
    // ISSUE #2 Hide the command name label.
    // Add property to layout to set label position to HIDDEN.
    // See "Adding widgets to layouts" in Shuffleboard docs.
  }

  public void startLoopTime() {
    startTime = Timer.getFPGATimestamp();
  }

  public final boolean getEverBrownOut() {
    return this.everBrownout;
  }
}
