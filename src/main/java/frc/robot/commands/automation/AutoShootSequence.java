package frc.robot.commands.automation;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;

/**
 * Represents an automated shooting sequence for a robot. This class extends the
 * SequentialCommandGroup class. It contains a series of commands that are executed sequentially to
 * perform the shooting sequence in auto.
 *
 * @param angle A DoubleSupplier that provides the angle for the shooter.
 * @param velocity A DoubleSupplier that provides the velocity for the shooter.
 * @param restingAngle The resting angle for the shooter after the shooting sequence is completed.
 */
public class AutoShootSequence extends SequentialCommandGroup {

  public AutoShootSequence(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      DoubleSupplier slapperAngle,
      double slapperRestingPosition) {
    addCommands(
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())),
        shooter.speedUpShooter(velocity, shooterSequenceAcceleration),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeedSupplier(velocity),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        intake.startFeedingCommand(feedVelocity, feedAcceleration),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        new StopShoot(restingAngle, slapperRestingPosition));
  }

  public AutoShootSequence(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      double indexerVelocity,
      DoubleSupplier slapperAngle,
      double slapperRestingPosition) {
    addCommands(
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())),
        shooter.speedUpShooterSlow(velocity, shooterSequenceAcceleration),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeedSupplier(velocity),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        intake.startFeedingCommand(feedVelocity, feedAcceleration),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        // shooter.waitUntilRingLeft(),
        new StopShoot(restingAngle, slapperRestingPosition));
  }

  public AutoShootSequence(
      DoubleSupplier angle,
      DoubleSupplier velocity,
      double restingAngle,
      DoubleSupplier slapperAngle,
      double slapperRestingPosition,
      double timeout) {
    addCommands(
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())),
        shooter.speedUpShooter(velocity, shooterSequenceAcceleration),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeedSupplier(() -> velocity.getAsDouble() * 0.75),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeedSupplier(velocity),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        intake.startFeedingCommand(feedVelocity, feedAcceleration),
        new WaitCommand(timeout).raceWith(shooter.waitUntilRingLeft()),
        new StopShoot(restingAngle, slapperRestingPosition));
  }
}
