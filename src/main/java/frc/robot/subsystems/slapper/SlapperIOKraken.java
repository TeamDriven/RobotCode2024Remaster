package frc.robot.subsystems.slapper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SlapperIOKraken implements SlapperIO {
  private TalonFX slapperMotor;
  private DutyCycleEncoder throughboreEncoder;
  // private Encoder quadrutureEncoder = new Encoder(6, 5);

  MotionMagicVoltage motionMagicControl;
  VoltageOut voltageControl;
  NeutralOut stopMode;

  public SlapperIOKraken(int motorID, int encoderID) {
    slapperMotor = new TalonFX(motorID);
    throughboreEncoder = new DutyCycleEncoder(encoderID);

    motionMagicControl = new MotionMagicVoltage(0, true, -0.65, 0, false, false, false);

    voltageControl = new VoltageOut(0, true, false, false, false);

    stopMode = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    slapperMotor.setPosition(0);

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 25;

    configs.MotionMagic.MotionMagicCruiseVelocity = 10;
    configs.MotionMagic.MotionMagicAcceleration = 35;
    configs.MotionMagic.MotionMagicJerk = 40;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.kP = 25; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.3; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = slapperMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }

  public void updateInputs(SlapperIOInputs inputs) {
    inputs.motorPosition = slapperMotor.getPosition().getValueAsDouble();
    inputs.motorVelocity = slapperMotor.getVelocity().getValueAsDouble();
    inputs.encoderAngle = throughboreEncoder.getAbsolutePosition() / SlapperConstants.rotationsPerDegree - SlapperConstants.slapperOffset;;
  }

  public void runVoltage(double speed) {
    slapperMotor.set(speed);
  }

  public void stopMotor() {
    slapperMotor.setControl(stopMode);
  }

  public void runMotorToPosition(double desiredPos, double feedForward, PIDController posPID) {
    double power = posPID.calculate(getPosition(), desiredPos) + feedForward;
    power = MathUtil.clamp(power, -1.5, 2);

    slapperMotor.setControl(voltageControl.withOutput(power));
  }

  public double getPosition() {
    // return throughboreEncoder.getDistance() - slapperOffset;
    // return slapperMotor.getPosition().getValueAsDouble() / slapperTicksPerDegree;
    return throughboreEncoder.getAbsolutePosition() / SlapperConstants.rotationsPerDegree - SlapperConstants.slapperOffset;
  }

  public void resetPosition() {
    slapperMotor.setPosition(getPosition() * SlapperConstants.rotationsPerDegree);
  }
}
