package frc.robot.subsystems.actuation;

import static frc.robot.subsystems.actuation.ActuationConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ActuationIOFalcon500 implements ActuationIO {
  private TalonFX actuationMotor;
  private DutyCycleEncoder throughboreEncoder;

  private MotionMagicVoltage motionMagicControl;
  private VoltageOut voltageOut;
  private NeutralOut stopMode;

  public ActuationIOFalcon500(int motorID, int encoderChannel) {
    actuationMotor = new TalonFX(motorID);
    throughboreEncoder = new DutyCycleEncoder(encoderChannel);

    motionMagicControl = new MotionMagicVoltage(0, true, -0.3, 0, false, false, false);
    voltageOut = new VoltageOut(0, true, false, false, false);
    stopMode = new NeutralOut();

    syncPosition(throughboreEncoder.getAbsolutePosition() / encoderRotationsPerDegree - offset);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.05;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 30;
    configs.MotionMagic.MotionMagicAcceleration = 50;
    configs.MotionMagic.MotionMagicJerk = 75;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 20; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.1; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = actuationMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void updateInputs(ActuationIOInputs inputs) {
    inputs.encoderAngle =
        throughboreEncoder.getAbsolutePosition() / encoderRotationsPerDegree - offset;
    inputs.motorAngle = actuationMotor.getPosition().getValueAsDouble() / motorRotationsPerDegree;
  }

  @Override
  public void setAngle(double angle) {
    actuationMotor.setControl(motionMagicControl.withPosition(angle * motorRotationsPerDegree));
  }

  @Override
  public void runVoltage(double voltage) {
    actuationMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void stopMotor() {
    actuationMotor.setControl(stopMode);
  }

  @Override
  public void syncPosition(double encoderAngle) {
    actuationMotor.setPosition(encoderAngle * motorRotationsPerDegree);
  }
}
