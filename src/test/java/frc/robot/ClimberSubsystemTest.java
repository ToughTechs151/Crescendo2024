// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class ClimberSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private ClimberSubsystem climber;
  private ClimberSubsystem.Hardware climberHardware;
  private CANSparkMax mockMotorLeft;
  private CANSparkMax mockMotorRight;
  private RelativeEncoder mockEncoderLeft;
  private RelativeEncoder mockEncoderRight;
  private Relay relayLeft;
  private Relay relayRight;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotorLeft = mock(CANSparkMax.class);
    mockMotorRight = mock(CANSparkMax.class);
    mockEncoderLeft = mock(RelativeEncoder.class);
    mockEncoderRight = mock(RelativeEncoder.class);
    relayLeft = mock(Relay.class);
    relayRight = mock(Relay.class);

    // Reset preferences to default values so test results are consistent
    RobotPreferences.resetAllPreferences();

    // Create subsystem object using mock hardware
    climberHardware =
        new ClimberSubsystem.Hardware(
            mockMotorLeft,
            mockMotorRight,
            mockEncoderLeft,
            mockEncoderRight,
            relayLeft,
            relayRight);
    climber = new ClimberSubsystem(climberHardware);
  }

  @AfterEach
  public void closeClimber() {
    climber.close(); // motor is closed from the climber close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotorLeft).setVoltage(0.0);
    verify(mockMotorRight).setVoltage(0.0);
    assertThat(climber.getLeftVoltageCommand()).isZero();
    assertThat(climber.getRightVoltageCommand()).isZero();

    // Position should be set to starting position
    assertThat(climber.getMeasurementLeft()).isEqualTo(ClimberConstants.CLIMBER_OFFSET_METERS);
    assertThat(climber.getMeasurementRight()).isEqualTo(ClimberConstants.CLIMBER_OFFSET_METERS);
  }

  @Test
  @DisplayName("Test retract command and disable.")
  void testMoveCommand() {

    // Create a command to retract the climber then initialize
    Command retractCommand =
        climber.moveToPosition(ClimberConstants.CLIMBER_RETRACT_POSITION_METERS);
    retractCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    climber.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        ClimberConstants.CLIMBER_RETRACT_POSITION_METERS,
        telemetryDoubleMap.get("Climber Goal"),
        DELTA);

    // Execute the command to run the controller
    retractCommand.execute();
    climber.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isPositive();
    assertThat(telemetryDoubleMap.get("Climber Right Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Climber Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    climber.disable();
    climber.periodic();
    readTelemetry();
    verify(mockMotorLeft, times(2)).setVoltage(0.0);
    verify(mockMotorRight, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isZero();
    assertThat(telemetryDoubleMap.get("Climber Right Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Climber Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test Motors and Encoders Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrent = -3.3;
    when(mockMotorLeft.getOutputCurrent()).thenReturn(fakeCurrent);
    when(mockMotorRight.getOutputCurrent()).thenReturn(fakeCurrent);
    final double fakePosition = 1.5;
    when(mockEncoderLeft.getPosition()).thenReturn(fakePosition);
    when(mockEncoderRight.getPosition()).thenReturn(fakePosition);
    final double fakeVelocity = 0.123;
    when(mockEncoderLeft.getVelocity()).thenReturn(fakeVelocity);
    when(mockEncoderRight.getVelocity()).thenReturn(fakeVelocity);

    // The two motor voltages should be set twice for each motor: once to 0 when configured and once
    //  to a
    // positive value when controller is run.
    Command moveCommand =
        climber.moveToPosition(Constants.ClimberConstants.CLIMBER_EXTEND_POSITION_METERS);
    moveCommand.initialize();
    moveCommand.execute();
    verify(mockMotorLeft, times(2)).setVoltage(anyDouble());
    verify(mockMotorLeft).setVoltage(0.0);
    verify(mockMotorLeft, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    verify(mockMotorRight, times(2)).setVoltage(anyDouble());
    verify(mockMotorRight).setVoltage(0.0);
    verify(mockMotorRight, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    // Test position measurements from the encoder
    assertThat(climber.getMeasurementLeft())
        .isEqualTo(ClimberConstants.CLIMBER_OFFSET_METERS + fakePosition);

    assertThat(climber.getMeasurementRight())
        .isEqualTo(ClimberConstants.CLIMBER_OFFSET_METERS + fakePosition);

    // Check that telemetry was sent to dashboard
    climber.periodic();
    readTelemetry();

    assertEquals(fakeCurrent, telemetryDoubleMap.get("Climber Left Current"), DELTA);
    assertEquals(
        ClimberConstants.CLIMBER_OFFSET_METERS + fakePosition,
        telemetryDoubleMap.get("Climber Left Position"),
        DELTA);
    if (Constants.SD_SHOW_CLIMBER_EXTENDED_LOGGING_DATA) {
      assertEquals(fakeVelocity, telemetryDoubleMap.get("Climber Left Velocity"), DELTA);
    }

    assertEquals(fakeCurrent, telemetryDoubleMap.get("Climber Right Current"), DELTA);
    assertEquals(
        ClimberConstants.CLIMBER_OFFSET_METERS + fakePosition,
        telemetryDoubleMap.get("Climber Right Position"),
        DELTA);
    if (Constants.SD_SHOW_CLIMBER_EXTENDED_LOGGING_DATA) {
      assertEquals(fakeVelocity, telemetryDoubleMap.get("Climber Right Velocity"), DELTA);
    }
  }

  @Test
  @DisplayName("Test range limit and hold.")
  void testLimitAndHoldCommand() {

    // Try a command to move the climber above the limit
    Command moveCommand =
        climber.moveToPosition(Constants.ClimberConstants.CLIMBER_MAX_PULL_METERS + 0.1);
    moveCommand.initialize();
    climber.periodic();
    readTelemetry();
    assertEquals(
        ClimberConstants.CLIMBER_MAX_PULL_METERS, telemetryDoubleMap.get("Climber Goal"), DELTA);

    // Verify that the hold command runs the controller
    Command moveCommandHigh =
        climber.moveToPosition(Constants.ClimberConstants.CLIMBER_RETRACT_POSITION_METERS);
    Command holdCommand = climber.holdPosition();
    // Initialize to set goal but don't execute so hold can be checked
    moveCommandHigh.initialize();
    holdCommand.execute();
    climber.periodic();
    readTelemetry();

    // Motor command should be positive to pull the robot up with the climber mechanisms.
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isPositive();
    assertThat(telemetryDoubleMap.get("Climber Right Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Climber Enabled")).isTrue();
  }

  // ---------- Utility Functions --------------------------------------

  /* Read in telemetry values from the network table and store in maps */
  private int readTelemetry() {
    NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    Set<String> telemetryKeys = telemetryTable.getKeys();

    for (String keyName : telemetryKeys) {
      NetworkTableType entryType = telemetryTable.getEntry(keyName).getType();

      if (entryType == NetworkTableType.kDouble) {
        telemetryDoubleMap.put(keyName, telemetryTable.getEntry(keyName).getDouble(-1));
      } else if (entryType == NetworkTableType.kBoolean) {
        telemetryBooleanMap.put(keyName, telemetryTable.getEntry(keyName).getBoolean(false));
      }
    }
    return telemetryKeys.size();
  }
}
