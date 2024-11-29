package frc.robot.commands.automation;

import static frc.robot.subsystems.slapper.SlapperConstants.*;
import static frc.robot.Subsystems.actuation;
import static frc.robot.Subsystems.angleController;
import static frc.robot.Subsystems.shooter;
import static frc.robot.Subsystems.slapper;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.angleController.AngleControllerConstants;

/**
 * This class represents a command group that zeros the angle of the shooter. It consists of a
 * sequence of commands that perform the necessary actions to zero the angle.
 */
public class ZeroAngle extends SequentialCommandGroup {

  public ZeroAngle() {
    addCommands(
        // new InstantCommand(actuation::stopMotor),
        new InstantCommand(shooter::stopMotors),
        new InstantCommand(() -> slapper.setPosition(slapperRestingPosition)),
        new WaitCommand(0.5),
        angleController.waitUntilPressed().withTimeout(4),
        new InstantCommand(angleController::stopMotor),
        new InstantCommand(() -> angleController.setOnSensor()),
        new InstantCommand(actuation::syncPosition),
        new InstantCommand(() -> slapper.resetPosition()),
        // actuation.resetEncoderCommand(),
        new InstantCommand(() -> angleController.setPosition(AngleControllerConstants.restingPosition)),
        new InstantCommand(shooter::sitMode));
  }

  public ZeroAngle(double angleRestingPosition) {
    addCommands(
        // new InstantCommand(actuation::stopMotor),
        new InstantCommand(shooter::stopMotors),
        new InstantCommand(() -> slapper.setPosition(slapperRestingPosition)),
        new WaitCommand(0.25),
        angleController.waitUntilPressed().withTimeout(4),
        new InstantCommand(() -> angleController.stopMotor()),
        new InstantCommand(() -> angleController.setOnSensor()),
        new InstantCommand(actuation::syncPosition),
        new InstantCommand(slapper::resetPosition),
        // actuation.resetEncoderCommand(),
        new InstantCommand(() -> angleController.setPosition(AngleControllerConstants.restingPosition)),
        new InstantCommand(shooter::sitMode));
  }
}
