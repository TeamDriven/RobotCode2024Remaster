package frc.robot.subsystems.slapper;

import static frc.robot.subsystems.slapper.SlapperConstants.rotationsPerDegree;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The `Slapper` class represents a subsystem that controls the Slapper mechanism of the robot. It
 * provides methods to control the Slapper motor, set the position of the Slapper, and run the
 * Slapper at a specified voltage.
 */
public class Slapper extends SubsystemBase {
  private final SlapperIO slapperIO;
  private final SlapperIOInputsAutoLogged slapperInputs = new SlapperIOInputsAutoLogged();

  public enum controlMode {
    POSITION,
    VOLTAGE,
    STOP;
  }

  private controlMode mode;

  private double voltage = 0;
  private double desiredPos = 0;

  private PIDController posPID = new PIDController(0.05, 0, 0.0025);
  private double feedForward = -0.3;

  public Slapper(SlapperIO slapperIO) {
    this.slapperIO = slapperIO;
  }
  
  @Override
  public void periodic() {
    slapperIO.updateInputs(slapperInputs);
    Logger.processInputs("Actuation", slapperInputs);

    if (mode == controlMode.POSITION) {
      slapperIO.runMotorToPosition(desiredPos, feedForward, posPID);
    }else if (mode == controlMode.STOP) {
      slapperIO.stopMotor();
    } else if (mode == controlMode.VOLTAGE) {
      slapperIO.runVoltage(voltage);
    }
  }

  public Command waitUntilAtPosition() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return Math.abs(
                slapperInputs.motorPosition * rotationsPerDegree
                    - slapperIO.getPosition() * rotationsPerDegree) <= 0.1;
      }
    };
  }

  public void runVoltage(double voltage) {
    this.voltage = voltage;
    mode = controlMode.VOLTAGE;
  }

  public void setPosition(double position) {
    desiredPos = position;
    mode = controlMode.POSITION;
  }

  public void stopMotor() {
    mode = controlMode.STOP;
  }

  public void resetPosition() {
    setPosition(slapperIO.getPosition() * rotationsPerDegree);
  }
}
