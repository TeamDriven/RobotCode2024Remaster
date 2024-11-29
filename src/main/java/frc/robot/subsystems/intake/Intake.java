package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `Slapper` class represents a subsystem that controls the Slapper mechanism of the robot. It
 * provides methods to control the Slapper motor, set the position of the Slapper, and run the
 * Slapper at a specified voltage.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public enum controlMode {
    FEED,
    VOLTAGE,
    STOP;
  }

  private controlMode mode;

  private double voltage = 0;
  private double velocity = 0;
  private double acceleration = 0;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }
  
  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Actuation", intakeInputs);

    if (mode == controlMode.FEED) {
      intakeIO.feedMotor(velocity, acceleration);
    }else if (mode == controlMode.STOP) {
      intakeIO.stopMotor();
    } else if (mode == controlMode.VOLTAGE) {
      intakeIO.runVoltage(voltage);
    }
  }

  public Command waitUntilTripped() {
    return new Command() {
      @Override
      public void execute() {
        if (intakeIO.getNoteSensor()) {
          intakeIO.stopMotor();
        }
      }

      @Override
      public boolean isFinished() {
        return intakeIO.getNoteSensor();
      }
    };
  }
  
  public void feedMotor(double voltage, double acceleration) {
    this.velocity = velocity;
    this.acceleration = acceleration;
    mode = controlMode.FEED;
  }

  public void runVoltage(double voltage) {
    this.voltage = voltage;
    mode = controlMode.VOLTAGE;
  }

  public void stopMotor() {
    mode = controlMode.STOP;
  }
}
