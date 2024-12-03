package frc.robot.commands.automation;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.actuation.Actuation.position;
import java.util.function.DoubleSupplier;

/**
 * This class represents an autonomous shoot sequence without stopping. It extends the
 * SequentialCommandGroup class and contains a series of commands that are executed sequentially to
 * perform the shoot sequence.
 *
 * @param angle A DoubleSupplier that provides the angle for the angle controller.
 * @param velocity A DoubleSupplier that provides the velocity for the shooter.
 * @param restingAngle The resting angle for the actuation.
 */
public class AutoShootSequenceNoStop extends SequentialCommandGroup {

  public AutoShootSequenceNoStop(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      DoubleSupplier slapperAngle) {
    addCommands(
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())),
        new InstantCommand(() -> shooter.runShooter(velocity.getAsDouble(), shooterSequenceAcceleration)),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeed(velocity.getAsDouble() * 0.8),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeed(velocity.getAsDouble()),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        new InstantCommand(() -> intake.feedMotor(feedVelocity, feedAcceleration)),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        new InstantCommand(() -> intake.stopMotor()),
        new InstantCommand(() -> actuation.setPosition(position.TUCK)));
  }

  public AutoShootSequenceNoStop(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      double indexerVelocity,
      DoubleSupplier slapperAngle) {
    addCommands(
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())),
        new InstantCommand(() -> shooter.runShooterSlow(velocity.getAsDouble(), shooterSequenceAcceleration)),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeed(velocity.getAsDouble() * 0.8),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeed(velocity.getAsDouble()),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        new InstantCommand(() -> intake.feedMotor(feedVelocity, feedAcceleration)),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        new InstantCommand(() -> intake.stopMotor()),
        new InstantCommand(() -> actuation.setPosition(position.TUCK)));
  }
}
