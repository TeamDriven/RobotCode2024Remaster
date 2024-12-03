package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOKraken implements ClimberIO {
  private TalonFX climberMotor; // 18 id

  private VoltageOut voltageControl;
  private NeutralOut StopMode;

  public ClimberIOKraken(int motorID) {
    climberMotor = new TalonFX(motorID);

    voltageControl = new VoltageOut(0.0, true, false, false, false);
    StopMode = new NeutralOut();

    resetClimber();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configs.MotionMagic.MotionMagicCruiseVelocity = 130;
    configs.MotionMagic.MotionMagicAcceleration = 250;
    configs.MotionMagic.MotionMagicJerk = 350;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 10; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = climberMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.position = climberMotor.getPosition().getValueAsDouble();
  }

  public void runVoltage(double voltage) {
    if (climberMotor.getPosition().getValueAsDouble() < ClimberConstants.minHeight && voltage < 0) {
      stopMotor();
    } else if (climberMotor.getPosition().getValueAsDouble() > ClimberConstants.maxHeight
        && voltage > 0) {
      stopMotor();
    } else {
      runVoltage(voltage);
    }
  }

  public void runUnsafeVoltage(double voltage) {
    climberMotor.setControl(voltageControl.withOutput(voltage));
  }

  public void stopMotor() {
    climberMotor.setControl(StopMode);
  }

  public void resetClimber() {
    climberMotor.setPosition(0);
  }
}
