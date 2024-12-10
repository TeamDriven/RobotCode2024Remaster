package frc.robot.subsystems.angleController;

import static frc.robot.subsystems.angleController.AngleControllerConstants.rotationsPerDegree;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AngleController.AngleControllerIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/** The AngleController class represents a subsystem that controls the angle of the shooter. */
public class AngleController extends SubsystemBase {
  private final AngleControllerIO angleControllerIO;
  private final AngleControllerIOInputsAutoLogged angleControllerInputs =
      new AngleControllerIOInputsAutoLogged();

  private double position = 0;

  public AngleController(AngleControllerIO angleControllerIO) {
    this.angleControllerIO = angleControllerIO;
  }

  @Override
  public void periodic() {
    angleControllerIO.updateInputs(angleControllerInputs);
    Logger.processInputs("AngleController", angleControllerInputs);

    if (position != angleControllerIO.getPosition()) {
      angleControllerIO.stopMotor();
    } else {
      angleControllerIO.setPosition(position);
    }
  }

  public Command waitUntilAtPosition() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return Math.abs(
                angleControllerInputs.motorPosition * rotationsPerDegree
                    - angleControllerIO.getPosition() * rotationsPerDegree)
            <= 0.1;
      }
    };
  }

  public Command waitUntilPressed() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return angleControllerIO.getSensor();
      }
    };
  }

  public void stopMotor() {
    angleControllerIO.stopMotor();
  }

  public void setOnSensor() {
    angleControllerIO.setOnSensor();
  }

  public void setPosition(double pos) {
    this.position = pos;
  }
}
