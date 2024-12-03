package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ShooterIOKraken implements ShooterIO {
  // Distance, Angle, Speed
  private static final Double[][] distanceMap = {
    {1.4, 1.0, 65.0},
    {1.6, 3.0, 65.0},
    {1.8, 6.0, 65.0},
    {1.9, 8.0, 65.0},
    {2.05, 10.0, 65.0},
    {2.25, 12.0, 65.0},
    {2.45, 15.0, 65.0},
    {2.6, 17.0, 65.0},
    {2.75, 19.0, 65.0},
    {2.9, 20.5, 65.0},
    {3.0, 23.5, 65.0},
    {3.2, 24.5, 65.0},
    {3.35, 24.75, 65.0},
    {3.5, 25.0, 65.0},
    {3.55, 25.0, 65.0},
    {3.6, 25.25, 65.0},
    {3.7, 26.0, 65.0},
    {3.78, 27.5, 65.0},
    {3.85, 27.75, 65.0},
    {4.03, 28.5, 65.0},
    {4.2, 29.25, 65.0},
    {4.35, 30.5, 65.0},
    {4.5, 31.25, 65.0},
    {4.65, 31.5, 65.0},
    {4.8, 31.75, 65.0},
    {5.0, 33.0, 65.0}
  };

  private TalonFX leftShooterMotor = new TalonFX(15);
  private TalonFX rightShooterMotor = new TalonFX(16);

  private DigitalInput noteExitSensor = new DigitalInput(2);

  VelocityVoltage velocityControl;
  VelocityVoltage slowVelocityControl;
  VoltageOut sitControl;
  NeutralOut stopMode;

  public ShooterIOKraken(int lMotorID, int rMotorID) {
    leftShooterMotor = new TalonFX(lMotorID);
    rightShooterMotor = new TalonFX(rMotorID);

    velocityControl = new VelocityVoltage(0, 0, true, 0.85, 0, false, false, true);

    slowVelocityControl =
        new VelocityVoltage(
            0, 0, true, 0.5, // 0.5
            1, false, false, true);

    sitControl = new VoltageOut(2, false, false, false, true);

    stopMode = new NeutralOut();

    sitMode();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 45;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.4; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.35; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    configs.Slot1.kP = 0.278; // 0.278 // An error of 1 rotation per second results in 2V output
    configs.Slot1.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot1.kD =
        0.0005; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot1.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightShooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftShooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.lSpeed = leftShooterMotor.getVelocity().getValueAsDouble();
    inputs.rSpeed = rightShooterMotor.getVelocity().getValueAsDouble();
  }

  public void sitMode() {
    leftShooterMotor.setControl(sitControl.withOutput(1.5));
    rightShooterMotor.setControl(sitControl.withOutput(1.5));
    // leftShooterMotor.setControl(stopMode);
    // rightShooterMotor.setControl(stopMode);
  }

  public void stopMotors() {
    leftShooterMotor.setControl(stopMode);
    rightShooterMotor.setControl(stopMode);
  }

  public void runShooter(double velocity, double acceleration) {
    leftShooterMotor.setControl(
        velocityControl.withVelocity(velocity).withAcceleration(acceleration));

    rightShooterMotor.setControl(
        velocityControl.withVelocity(velocity).withAcceleration(acceleration));
  }

  public void runShooterSlow(double velocity, double acceleration) {
    leftShooterMotor.setControl(
        slowVelocityControl.withVelocity(velocity).withAcceleration(acceleration));

    rightShooterMotor.setControl(
        slowVelocityControl.withVelocity(velocity).withAcceleration(acceleration));
  }

  public boolean getNoteSensor() {
    return noteExitSensor.get();
  }
}
