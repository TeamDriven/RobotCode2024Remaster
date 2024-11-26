package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * A command group that stops the shooter, indexer motor, intake, and sets the angle of the shooter.
 */
public class StopShoot extends ParallelCommandGroup {

  /**
   * Constructs a new StopShoot command group with the specified angle.
   *
   * @param angle the angle to set the shooter to
   */
  public StopShoot(double angle, double slapperRestingPosition) {
    addCommands(
        new InstantCommand(shooter::sitMode, shooter),
        new InstantCommand(indexer::stopIndexerMotor, indexer),
        intake.stopIntakeCommand(),
        new InstantCommand(() -> angleController.setPosition(angle)),
        slapper.setPositionCommand(slapperRestingPosition));
  }
}
