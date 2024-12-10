package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOKraken implements IntakeIO {
  private TalonFX intakeMotor;

  private DigitalInput rightNoteSensor;
  private DigitalInput leftNoteSensor;

  VelocityVoltage velocityControlFeed;
  VoltageOut voltageControl;
  NeutralOut stopMode;

  public IntakeIOKraken(int motorID, int rightSensorID, int leftSensorID) {
    intakeMotor = new TalonFX(motorID);

    rightNoteSensor = new DigitalInput(rightSensorID);
    leftNoteSensor = new DigitalInput(leftSensorID);

    // velocityControlFeed = new VelocityVoltage(0, 0, true, 0.5, 1, false, false, false);
    velocityControlFeed = new VelocityVoltage(0);

    voltageControl = new VoltageOut(0);

    stopMode = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    configs.Slot1.kP = 4; // An error of 1 rotation per second results in 2V output
    configs.Slot1.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot1.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot1.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorVelocity = intakeMotor.getPosition().getValueAsDouble();
  }

  public void feedMotor(double velocity, double acceleration) {
    intakeMotor.setControl(
        velocityControlFeed.withVelocity(velocity).withAcceleration(acceleration).withEnableFOC(true).withFeedForward(0.5).withSlot(1));
  }

  public void stopMotor() {
    intakeMotor.setControl(stopMode);
  }

  public boolean getNoteSensor() {
    return leftNoteSensor.get() || rightNoteSensor.get();
  }

  public void runVoltage(double voltage) {
    intakeMotor.setControl(voltageControl.withOutput(voltage));
  }
}
