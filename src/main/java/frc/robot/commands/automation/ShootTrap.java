package frc.robot.commands.automation;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.slapper.SlapperConstants.*;
// import static frc.robot.Constants.SlapperConstants.slapperTrapPosition;
import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.angleController.AngleControllerConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.angleController.AngleControllerConstants;

/** A command group that represents the sequence of commands to shoot at the trap. */
public class ShootTrap extends SequentialCommandGroup {

  public ShootTrap() {
    addCommands(
        // new PrintCommand("Angle: " + angle.getAsDouble()),
        // new PrintCommand("Speed: " + velocity.getAsDouble())
        new InstantCommand(() -> angleController.setPosition(trapAngle)),
        new InstantCommand(() -> slapper.setPosition(slapperRestingPosition)),
        shooter.speedUpShooter(() -> trapSpeed, shooterSequenceAcceleration),
        angleController.waitUntilAtPosition(),
        shooter.checkIfAtSpeedSupplier(() -> trapSpeed * 0.8),
        new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration)),
        shooter.checkIfAtSpeedSupplier(() -> trapSpeed),
        indexer.checkIfAtSpeed(() -> indexerVelocity),
        actuation.waitUntilAtPosition(),
        new InstantCommand(() -> intake.feedMotor(feedVelocity, feedAcceleration)),
        new WaitCommand(0.5).raceWith(shooter.waitUntilRingLeft()),
        new StopShoot(AngleControllerConstants.restingPosition, slapperRestingPosition));
  }
}
