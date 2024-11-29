package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerIOKraken implements IndexerIO {
    private TalonFX indexerMotor;

    VelocityVoltage velocityControl;
    NeutralOut stopMode;

  public IndexerIOKraken(int motorID) {
    indexerMotor = new TalonFX(motorID);

    velocityControl = new VelocityVoltage(0, 0, true, 0.6, 0, false, false, false);

    stopMode = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.6; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexerMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.speed = indexerMotor.getVelocity().getValueAsDouble();
  }

  public void runIndexer(double velocity, double acceleration) {
    indexerMotor.setControl(velocityControl.withVelocity(velocity).withAcceleration(acceleration));
  }

  public void stopMotor() {
    indexerMotor.setControl(stopMode);
  }

  public double getVelocity() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }
}
