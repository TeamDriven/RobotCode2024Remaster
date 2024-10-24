// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.actuation;

import static frc.robot.subsystems.actuation.ActuationConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Actuation subsystem controls the actuation motor that moves a mechanism to a specific
 * position.
 */
public class Actuation extends SubsystemBase {
  private final ActuationIO actuationIO;
  private final ActuationIOInputsAutoLogged actuationInputs = new ActuationIOInputsAutoLogged();

  public enum position {
    MANUAL(0),
    STOP(0),
    START(startPosition),
    TUCK(tuckPosition),
    PICK_UP(pickUpPosition);

    public double angle;

    position(double angle) {
      this.angle = angle;
    }
  }

  private position currentPosition = position.START;

  private double manualVoltage = 0;

  private PIDController posPID = new PIDController(1.5, 0, 0.01);

  /** Creates a new Actuation. */
  public Actuation(ActuationIO actuationIO) {
    this.actuationIO = actuationIO;

    posPID.disableContinuousInput();
  }

  @Override
  public void periodic() {
    actuationIO.updateInputs(actuationInputs);

    switch (currentPosition) {
      case MANUAL:
        actuationIO.runVoltage(manualVoltage);
        return;

      case STOP:
        actuationIO.stopMotor();
        return;

      case START:
        posPID.setTolerance(1);
        break;

      case TUCK:
        posPID.setTolerance(1);
        break;

      case PICK_UP:
        posPID.setTolerance(2.5);
        break;
    }

    double desiredAngle = currentPosition.angle;

    // Needs to happen regardless since pid uses deltas
    double power =
        posPID.calculate(
            actuationInputs.motorAngle * motorRotationsPerDegree,
            desiredAngle * motorRotationsPerDegree);
    power = MathUtil.clamp(power, -3, 2.5);
    if (posPID.atSetpoint()) {
      actuationIO.setAngle(desiredAngle);
    } else {
      actuationIO.runVoltage(power);
    }
  }

  public void setPosition(position pos) {
    currentPosition = pos;
  }

  public Command waitUntilAtPosition() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return Math.abs(
                actuationInputs.motorAngle * motorRotationsPerDegree
                    - currentPosition.angle * motorRotationsPerDegree)
            <= 0.5;
      }
    };
  }

  /** Reset the actuation motor encoder to it's starting position */
  public void syncPosition() {
    actuationIO.syncPosition(actuationInputs.encoderAngle);
  }
}
