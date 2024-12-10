package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.angleController.AngleControllerConstants;

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
        new InstantCommand(shooter::stopMotors, shooter),
        new InstantCommand(() -> indexer.stopMotor()),
        new InstantCommand(() -> intake.stopMotor()),
        new InstantCommand(() -> angleController.setPosition(angle)),
        new InstantCommand(
            () -> angleController.setPosition(AngleControllerConstants.restingPosition)));
  }
}
