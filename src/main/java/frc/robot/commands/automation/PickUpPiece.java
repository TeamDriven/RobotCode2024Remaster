package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShakeController;
import frc.robot.subsystems.actuation.Actuation.position;

/**
 * A command group that represents the process of picking up a piece. This command group
 * sequentially executes a series of commands to pick up a piece, including positioning the
 * actuation mechanism, running the intake, and tucking the actuation mechanism. It also includes a
 * shake controller to add a shaking motion after picking up the piece.
 *
 * @param voltage The voltage to run the intake at.
 */
public class PickUpPiece extends SequentialCommandGroup {

  public PickUpPiece(double voltage) {
    addCommands(
        new InstantCommand(() -> actuation.setPosition(position.PICK_UP)),
        actuation.waitUntilAtPosition(),
        new InstantCommand(() -> intake.runVoltage(voltage)),
        intake.waitUntilTripped(),
        new InstantCommand(() -> intake.stopMotor()),
        new InstantCommand(() -> actuation.setPosition(position.TUCK)),
        new ShakeController(1.0, 1.0));
  }
}
