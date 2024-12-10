package frc.robot.subsystems.angleController;

import static frc.robot.subsystems.angleController.AngleControllerConstants.rotationsPerDegree;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class AngleControllerIOKraken implements AngleControllerIO {
  private TalonFX AngleControllerMotor; // 19

  private DigitalInput zeroSensor; // 4

  MotionMagicVoltage motionMagicControl;
  NeutralOut stopMode;

  public AngleControllerIOKraken(int motorID, int sensorID) {
    AngleControllerMotor = new TalonFX(motorID);

    zeroSensor = new DigitalInput(sensorID);

    motionMagicControl = new MotionMagicVoltage(0);
    stopMode = new NeutralOut();

    resetEncoder();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 15;
    configs.MotionMagic.MotionMagicAcceleration = 20;
    configs.MotionMagic.MotionMagicJerk = 50;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 60; // 25 // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.5; // 4 // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = AngleControllerMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void updateInputs(AngleControllerIOInputs inputs) {
    inputs.motorPosition = AngleControllerMotor.getPosition().getValueAsDouble();
    inputs.sensorIsTripped = zeroSensor.get();
  }

  public void setPosition(double position) {
    AngleControllerMotor.setControl(
        motionMagicControl.withPosition(position * AngleControllerConstants.rotationsPerDegree).withFeedForward(-0.1).withSlot(0));
  }

  public void stopMotor() {
    AngleControllerMotor.setControl(stopMode);
  }

  public void setOnSensor() {
    AngleControllerMotor.setPosition(36.0 * rotationsPerDegree);
  }

  public void resetEncoder() {
    AngleControllerMotor.setPosition(
        AngleControllerConstants.startingPosition * AngleControllerConstants.rotationsPerDegree);
  }
}
