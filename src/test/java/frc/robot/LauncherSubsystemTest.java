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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.AdditionalMatchers;

class LauncherSubsystemTest {
  private static final double DELTA = 5e-3;
  private Map<String, Double> telemetryDoubleMap = new HashMap<>();
  private Map<String, Boolean> telemetryBooleanMap = new HashMap<>();

  private LauncherSubsystem.Hardware launcherHardware;
  private LauncherSubsystem launcher;
  private CANSparkMax mockMotorLeft;
  private CANSparkMax mockMotorRight;
  private RelativeEncoder mockEncoderLeft;
  private RelativeEncoder mockEncoderRight;

  @BeforeEach
  public void initEach() {
    // Create mock hardware devices
    mockMotorLeft = mock(CANSparkMax.class);
    mockMotorRight = mock(CANSparkMax.class);
    mockEncoderLeft = mock(RelativeEncoder.class);
    mockEncoderRight = mock(RelativeEncoder.class);

    // Reset preferences to default values so test results are consistent
    RobotPreferences.resetPreferences();

    // Create subsystem object using mock hardware
    launcherHardware =
        new LauncherSubsystem.Hardware(
            mockMotorRight, mockMotorLeft, mockEncoderRight, mockEncoderLeft);
    launcher = new LauncherSubsystem(launcherHardware);
  }

  @AfterEach
  public void closeLauncher() {
    launcher.close(); // motor is closed from the launcher close method
  }

  @Test
  @DisplayName("Test constructor and initialization.")
  void testConstructor() {
    // We haven't enabled it yet, so command to motor and saved value should be zero.
    verify(mockMotorLeft).setVoltage(0.0);
    assertThat(launcher.getLauncherVoltageCommandLeft()).isZero();
    assertThat(launcher.getLauncherVoltageCommandRight()).isZero();
  }

  @Test
  @DisplayName("Test run command and disable.")
  void testMoveCommand() {

    // Create a command to run the launcher then initialize
    Command runLauncherCommand = launcher.runLauncher(LauncherConstants.LAUNCHER_FULL_SPEED);
    runLauncherCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    launcher.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        LauncherConstants.LAUNCHER_FULL_SPEED,
        telemetryDoubleMap.get("Launcher Left Setpoint"),
        DELTA);
    assertEquals(
        -LauncherConstants.LAUNCHER_FULL_SPEED,
        telemetryDoubleMap.get("Launcher Right Setpoint"),
        DELTA);

    // Execute the command to run the controller
    runLauncherCommand.execute();
    launcher.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Launcher Left Voltage")).isPositive();
    assertThat(telemetryDoubleMap.get("Launcher Right Voltage")).isNegative();
    assertThat(telemetryBooleanMap.get("Launcher Enabled")).isTrue();

    // When disabled Motor should be commanded to zero
    launcher.disableLauncher();
    launcher.periodic();
    readTelemetry();
    verify(mockMotorLeft, times(2)).setVoltage(0.0);
    verify(mockMotorRight, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Launcher Left Voltage")).isZero();
    assertThat(telemetryDoubleMap.get("Launcher Right Voltage")).isZero();
    assertThat(telemetryBooleanMap.get("Launcher Enabled")).isFalse();
  }

  @Test
  @DisplayName("Test Motor and Encoder Sensors.")
  void testSensors() {

    // Set values for mocked sensors
    final double fakeCurrentLeft = -3.3;
    final double fakeCurrentRight = 4.4;
    when(mockMotorLeft.getOutputCurrent()).thenReturn(fakeCurrentLeft);
    when(mockMotorRight.getOutputCurrent()).thenReturn(fakeCurrentRight);
    final double fakeVelocityLeft = 123.5;
    final double fakeVelocityRight = -234.5;
    when(mockEncoderLeft.getVelocity()).thenReturn(fakeVelocityLeft);
    when(mockEncoderRight.getVelocity()).thenReturn(fakeVelocityRight);

    // The motor voltages should be set twice: once to 0 when configured and once to a
    // positive value for left and negative value for right when controller is run.
    Command runLauncherCommand = launcher.runLauncher(LauncherConstants.LAUNCHER_FULL_SPEED);
    runLauncherCommand.initialize();
    runLauncherCommand.execute();
    verify(mockMotorLeft, times(2)).setVoltage(anyDouble());
    verify(mockMotorLeft).setVoltage(0.0);
    verify(mockMotorLeft, times(1)).setVoltage(AdditionalMatchers.gt(0.0));
    verify(mockMotorRight, times(2)).setVoltage(anyDouble());
    verify(mockMotorRight).setVoltage(0.0);
    verify(mockMotorRight, times(1)).setVoltage(AdditionalMatchers.lt(0.0));

    // Check that telemetry was sent to dashboard
    launcher.periodic();
    readTelemetry();
    assertEquals(fakeCurrentLeft, telemetryDoubleMap.get("Launcher Left Current"), DELTA);
    assertEquals(fakeVelocityLeft, telemetryDoubleMap.get("Launcher Left Speed"), DELTA);
    assertEquals(fakeCurrentRight, telemetryDoubleMap.get("Launcher Right Current"), DELTA);
    assertEquals(fakeVelocityRight, telemetryDoubleMap.get("Launcher Right Speed"), DELTA);
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
