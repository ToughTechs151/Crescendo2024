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
    climberHardware =
        new ClimberSubsystem.Hardware(
            mockMotorLeft, mockMotorRight, mockEncoderLeft, mockEncoderRight);
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
    assertThat(climber.getVoltageCommand()).isZero();

    // Position should be set to starting position
    assertThat(climber.getMeasurementLeft()).isEqualTo(ClimberConstants.CLIMBER_OFFSET_RADS);
    assertThat(climber.getMeasurementRight()).isEqualTo(ClimberConstants.CLIMBER_OFFSET_RADS);
  }

  @Test
  @DisplayName("Test move command and disable.")
  void testMoveCommand() {

    // Create a command to move the elevator then initialize
    Command moveCommand = climber.moveToPosition(ClimberConstants.CLIMBER_LOW_POSITION);
    moveCommand.initialize();

    // Run the periodic method to generate telemetry and verify it was published
    climber.periodic();
    int numEntries = readTelemetry();
    assertThat(numEntries).isPositive();
    assertEquals(
        ClimberConstants.CLIMBER_LOW_POSITION, telemetryDoubleMap.get("Climber Goal"), DELTA);

    // Execute the command to run the controller
    moveCommand.execute();
    climber.periodic();
    readTelemetry();
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isPositive();
    assertThat(telemetryBooleanMap.get("Climber Enabled")).isTrue();

    // When disabled mMotor should be commanded to zero
    climber.disable();
    climber.periodic();
    readTelemetry();
    verify(mockMotorLeft, times(2)).setVoltage(0.0);
    verify(mockMotorRight, times(2)).setVoltage(0.0);
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isZero();
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
    Command moveCommand = climber.moveToPosition(Constants.ClimberConstants.CLIMBER_LOW_POSITION);
    moveCommand.initialize();
    moveCommand.execute();
    verify(mockMotorLeft, times(2)).setVoltage(anyDouble());
    verify(mockMotorLeft).setVoltage(0.0);
    verify(mockMotorLeft, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    verify(mockMotorRight, times(2)).setVoltage(anyDouble());
    verify(mockMotorRight).setVoltage(0.0);
    verify(mockMotorRight, times(1)).setVoltage(AdditionalMatchers.gt(0.0));

    // This unused code is provided as an example of looking for a specific value.
    // This value was cheated by running working code as an example since calculating actual
    // controller expected values is difficult.  Instead the test above just checks direction
    // of the command, and controller response tests are done in simulation by checking desired
    // response over time.
    //
    // final double expectedCommand = 0.34092;
    // verify(mockMotorLeft, times(1)).setVoltage(AdditionalMatchers.eq(expectedCommand, DELTA));
    //  verify(mockMotorRight, times(1)).setVoltage(AdditionalMatchers.eq(expectedCommand, DELTA));

    // Alternative method: capture values and then use them in a test criteria
    // ArgumentCaptor<Double> argument = ArgumentCaptor.forClass(Double.class);
    // verify(mockMotorLeft).setVoltage(argument.capture()); // Can use this if only called once
    // verify(mockMotorLeft, times(2)).setVoltage(argument.capture());
    // verify(mockMotorRight).setVoltage(argument.capture()); // Can use this if only called once
    // verify(mockMotorRight, times(2)).setVoltage(argument.capture());
    // assertEquals(expectedCommand, argument.getValue(), DELTA);

    // Test position measurements from the encoder
    assertThat(climber.getMeasurementLeft())
        .isEqualTo(ClimberConstants.CLIMBER_OFFSET_RADS + fakePosition);

    assertThat(climber.getMeasurementRight())
        .isEqualTo(ClimberConstants.CLIMBER_OFFSET_RADS + fakePosition);

    // Check that telemetry was sent to dashboard
    climber.periodic();
    readTelemetry();

    /*
    SmartDashboard.putBoolean("Climber Enabled", climberEnabled);
    SmartDashboard.putNumber("Climber Goal", climberController.getGoal().position);
    SmartDashboard.putNumber("Climber Left Position", getMeasurementLeft());
    SmartDashboard.putNumber("Climber Right Position", getMeasurementLeft());
    SmartDashboard.putNumber("Climber Left Velocity", encoderLeft.getVelocity());
    SmartDashboard.putNumber("Climber Right Velocity", encoderRight.getVelocity());
    SmartDashboard.putNumber("Climber Left Voltage", voltageCommand);
    SmartDashboard.putNumber("Climber Right Voltage", voltageCommand);
    SmartDashboard.putNumber("Climber Left Current", motorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Climber Right Current", motorRight.getOutputCurrent());
    SmartDashboard.putNumber("Climber Left Feedforward", newFeedforward);
    SmartDashboard.putNumber("Climber Right Feedforward", newFeedforward);
    SmartDashboard.putNumber("Climber Left PID output", output);
    SmartDashboard.putNumber("Climber Right PID output", output);
    SmartDashboard.putNumber("Climber Left SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Climber Right SetPt Vel", setpoint.velocity);
    SmartDashboard.putNumber("Climber Left SetPt Pos", setpoint.position);
    SmartDashboard.putNumber("Climber Right SetPt Vel", setpoint.velocity);

    */
    assertEquals(fakeCurrent, telemetryDoubleMap.get("Climber Left Current"), DELTA);
    assertEquals(
        ClimberConstants.CLIMBER_OFFSET_RADS + fakePosition,
        telemetryDoubleMap.get("Climber Left Position"),
        DELTA);
    assertEquals(fakeVelocity, telemetryDoubleMap.get("Climber Left Velocity"), DELTA);
  }

  @Test
  @DisplayName("Test range limit and hold.")
  void testLimitAndHoldCommand() {

    // Try a command to move the climber above the limit
    Command moveCommand =
        climber.moveToPosition(Constants.ClimberConstants.CLIMBER_MAX_HEIGHT_METERS + 0.1);
    moveCommand.initialize();
    climber.periodic();
    readTelemetry();
    assertEquals(
        ClimberConstants.CLIMBER_MAX_HEIGHT_METERS, telemetryDoubleMap.get("Climber Goal"), DELTA);

    // Verify that the hold command runs the controller
    Command moveCommandHigh =
        climber.moveToPosition(Constants.ClimberConstants.CLIMBER_HIGH_POSITION);
    Command holdCommand = climber.holdPosition();
    // Initialize to set goal but don't execute so hold can be checked
    moveCommandHigh.initialize();
    holdCommand.execute();
    climber.periodic();
    readTelemetry();

    // Motor command should be positive to move elevator up.
    assertThat(telemetryDoubleMap.get("Climber Left Voltage")).isPositive();
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
